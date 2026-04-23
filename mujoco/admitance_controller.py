class AdmittanceController:
    def __init__(self, mass, damping, kr_gain):
        self.virtual_mass = mass      # Virtual mass (low = robot feels light)
        self.virtual_dumping = damping   # Damping (prevents oscillations)
        self.residual_gain = kr_gain  # The gain for your residual r
        
        self.q_cmd = 0.0
        self.dq_cmd = 0.0
        self.last_time = 0.0

    def update(self, q_current, r_vector, time):
        """
        Switches to reflex mode once collision is detected.
        r_vector: The residual vector calculated from your observer.
        """
        # The reflex torque law: τ = KR * r
        tau_reflex = self.residual_gain * r_vector
        
        # Admittance Physics: M*ddq + B*dq = tau_reflex
        # Solving for acceleration: ddq = (tau_reflex - B*dq) / M
        ddq = (tau_reflex - (self.virtual_dumping * self.dq_cmd)) / self.virtual_mass

        dt = time - self.last_time 
        self.last_time = time
        
        # Integrate to get the next position command
        self.dq_cmd += ddq * dt
        self.q_cmd += self.dq_cmd * dt
        
        return q_current + self.q_cmd

import numpy as np

