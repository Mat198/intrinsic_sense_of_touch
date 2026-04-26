import mujoco
import mujoco.viewer
import numpy as np
np.set_printoptions(precision=3, suppress=True) # Debug

import time

from collision_detector import CollisionDetector
from admitance_controller import AdmittanceController

# Loading mujoco scene from the mujoco menagerie (couldn't find a better way than copy/paste)
model = mujoco.MjModel.from_xml_path('kuka_iiwa_14/scene.xml')
data = mujoco.MjData(model)

# Create the inertia matrix M
nv = model.nv
M_matrix = np.zeros((nv, nv))

# Collision detector. If the gain is too large the residual is almost equal torque but
# the noise is too high
collision_gain = 100
collision_threshold = 10 # Min residual value to consider a valid collision
collision_detector = CollisionDetector(
    gain=collision_gain, threshold=collision_threshold, joint_number=nv)

# Admitance controller. 
Kr = 0.001
admitance_controller = AdmittanceController(
    mass=5.0, damping=100.0, stiffness=2000, kr_gain=1.0)    

# Initial position
desired_q = np.array([0,-np.pi/6,0,np.pi/3,0,-np.pi/2,0])

# Controller switch variables
collision_avoid_mode = False

# Sets a timer to evade collision and avoid switch back to the default PD controller too fast
evading = False
evading_start_time = 0
last_time = 0
collision_evasion_duration = 3.0

# Default PD controller variables
Kp = 0.0001
Kd = 0.0001
error = 0
# Setting a different reference for pd so the robot doesn't snap imediatily
pd_reference = desired_q.copy()
pd_return_rate = 0.05  # rad/s max speed when returning to desired_q after evasion

# Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation started!")
    # Simulation loop
    while viewer.is_running():
        # Calculating time interval
        step_time = time.time()
        sim_time = data.time
        dt = sim_time - last_time
        last_time = sim_time

        # Update robot data
        joint_positions = data.qpos[:]
        joint_velocities = data.qvel[:]
        joint_torques = data.qfrc_actuator[:]
        mujoco.mj_fullM(model, M_matrix, data.qM) # Fill the inertia matrix M(q)
        
        # Calculating the residual and identifying collisions.
        momentum = M_matrix @ joint_velocities
        # alpha = C^T(q, q̇)q̇ − g(q), Coriolis and centrifugal - gravity vector
        alpha_model = data.qfrc_bias + data.qfrc_passive 
        collision_result, residuals = collision_detector.identify_collision(
            momentum, alpha_model, joint_torques, dt)

        # Controller switch logic
        if (sim_time < 1.0):
            # Setting the robot inital position
            data.ctrl[:] = desired_q
        else:
            # If not in collision, uses the default PD controller
            if not any(collision_result) and not evading:
                if collision_avoid_mode:
                    print("Normal operation mode")
                    collision_avoid_mode = False
                    pd_reference = joint_positions.copy()
                # Glide pd_reference toward desired_q at a bounded rate
                step = pd_return_rate * dt
                diff = desired_q - pd_reference
                pd_reference += np.clip(diff, -step, step)
                error = desired_q - joint_positions
                data.ctrl[:] = pd_reference + Kp * error - Kd * joint_velocities
            else:
                # During collision, try to avoid it
                if collision_avoid_mode and not evading:
                    print("Retrying to evade...")
                    evading = True
                    evading_start_time = sim_time
                if not collision_avoid_mode:
                    print("Collision avoidance operation mode")
                    collision_avoid_mode = True
                    evading = True
                    evading_start_time = sim_time
                    print("Starting evasion!")
                    admitance_controller.set_state(joint_positions, joint_velocities)
                    collision_detector.reset()
                # Evades for a while and them try to return to default mode
                if (sim_time - evading_start_time > collision_evasion_duration):
                    evading = False
                    print("Evasion ended!")
                data.ctrl[:] = admitance_controller.update(joint_positions, residuals, data.qfrc_bias, dt)

        # Advance the simulation and update the viewer
        mujoco.mj_step(model, data)
        viewer.sync()

        # Maintain real-time frequency
        time_until_next_step = model.opt.timestep - (time.time() - step_time)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)