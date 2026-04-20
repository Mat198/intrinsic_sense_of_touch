import mujoco
import mujoco.viewer
import numpy as np
import time

from collision_detector import CollisionDetector

# 1. Load the model
# Replace 'scene.xml' with the path to your actual file
model = mujoco.MjModel.from_xml_path('kuka_iiwa_14/scene.xml')
data = mujoco.MjData(model)

nv = model.nv
M_matrix = np.zeros((nv, nv))

collision_detector = CollisionDetector(100, 5, nv)

# 2. Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Start the simulation loop
    start = time.time()

    print("Simulation started!")
    
    while viewer.is_running():
        step_start = time.time()

        # --- STEP A: GET DATA FROM ROBOT ---
        # Get joint positions (qpos) and velocities (qvel)
        # MuJoCo stores these in contiguous arrays
        joint_positions = data.qpos[:]
        joint_velocities = data.qvel[:]
        joint_torques = data.actuator_force[:]

        mujoco.mj_fullM(model, M_matrix, data.qM) 
        momentum = M_matrix @ joint_velocities

        alpha_model = data.qfrc_bias + data.qfrc_passive

        collision_result = collision_detector.identify_collision(momentum, alpha_model, joint_torques, data.time)
        # print(f"Collision vector {collision_result}")

        # --- STEP B: CONTROL LOOP ---
        t = data.time
        data.ctrl[:] = [0,-np.pi/6,0,np.pi/3,0,-np.pi/2,0]

        # --- STEP C: ADVANCE SIMULATION ---
        # mj_step integrates the physics
        mujoco.mj_step(model, data)

        # --- STEP D: SYNC VIEWER ---
        # Update the visualizer with the new state
        viewer.sync()

        # Maintain real-time frequency
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)