# Intrinsic Sense of Touch

MuJoCo simulation replicating the collision detection and reaction system from:

> *Collision Detection and Safe Reaction with the DLR-III Lightweight Manipulator Arm*
> IROS, 2006. https://ieeexplore.ieee.org/abstract/document/4058607

The core idea: a robot arm can detect external contact forces using only its existing joint torque
sensors and a dynamic model — no dedicated tactile skin required.

The iiwa model used here is from the [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).

![](intrinsic_sense_of_touch.gif)

## How it works

### Collision detection — momentum-based residual observer

External forces are estimated by comparing the robot's actual generalized momentum against what the dynamic model predicts. The residual `r` is computed as:

```
r = K * [p - integral{(tau + r - alpha) dt}]
```

Where:
- `p = M(q) * q̇` — generalized momentum
- `tau` — measured joint torques (`qfrc_actuator`)
- `alpha = C(q,q̇)q̇ - g(q)` — Coriolis/centrifugal and gravity terms (`qfrc_bias`)
- `K` — observer gain matrix

When `|r|` exceeds a threshold, a collision is declared on that joint. 
See [collision_detector.py](mujoco/collision_detector.py).

### Admittance controller — collision reaction

On collision, the robot switches from position control to an admittance controller that makes the arm compliant and moves it away from the contact. The virtual dynamics are:

```
M_v * ddq_v + B_v * dq_v = tau_reflex + K_s * (q_v - q_actual)
```

Where:
- `tau_reflex = Kr * r` — force proportional to the residual
- `K_s` — virtual spring that anchors the trajectory near the current pose
- `M_v`, `B_v` — virtual mass and damping

Gravity compensation is applied as a position offset: `q_cmd = q_v + g(q) / K_actuator`.
See [admitance_controller.py](mujoco/admitance_controller.py).

The controller outputs a position command to the joints. This control interface was chosen so the code can be ported to a real robot that doesn't have a torque interface like iiwa.

### PD controller — normal operation

Outside of collision, a lightweight outer-loop PD holds `desired_q`. When returning from admittance
mode, the setpoint glides from the current position back to `desired_q` at a bounded rate to avoid
torque spikes. The inner actuator (stiffness 2000, damping 200) handles low-level tracking.
See [main.py](mujoco/main.py).

## Repository structure

```
mujoco/
  main.py                  — simulation loop, controller switch logic
  collision_detector.py    — momentum-based residual observer
  admitance_controller.py  — admittance controller for collision reaction
  kuka_iiwa_14/            — KUKA iiwa 14 model (MuJoCo Menagerie)
```

## Running

Requires MuJoCo and its Python bindings:

```bash
pip install mujoco
cd mujoco
python main.py
```

The simulation loads the KUKA iiwa 14 model, moves to a default pose for 1 second, then activates the controller loop. Push the arm in the viewer to trigger the collision reaction. To push the arm, double-click a link; it becomes slightly brighter, then ctrl + left-click to move it.

## Parameters

| Parameter | Location | Description |
|---|---|---|
| `collision_gain` | `main.py` | Observer gain `K`. Higher = faster response, more noise |
| `collision_threshold` | `main.py` | Min residual (N·m) to declare collision |
| `kr_gain` | `AdmittanceController` | Scales residual into reflex torque |
| `mass`, `damping`, `stiffness` | `AdmittanceController` | Virtual dynamics of the compliant behaviour |
| `pd_return_rate` | `main.py` | Max rad/s when gliding back to `desired_q` after evasion |

## Reference papers

- De Luca et al., *Sensorless_Robot_Collision_Detection_and_Hybrid_Force_Motion_Control*, IROS 2005
- De Luca et al., *Collision Detection and Safe Reaction with the DLR-III Lightweight Manipulator Arm*, IROS 2006
- Birjandi et al., *Collision Detection, Identification, and Localization on the DLR SARA Robot with Sensing Redundancy*, ICRA 2020
- Haddadin et al., *Intrinsic sense of touch for intuitive physical human-robot interaction*, Science Robotics 2024