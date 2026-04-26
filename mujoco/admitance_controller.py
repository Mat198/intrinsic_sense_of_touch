class AdmittanceController:
    def __init__(self, mass, damping, stiffness, kr_gain):
        self.virtual_mass = mass
        self.virtual_dumping = damping
        self.virtual_stiffness = stiffness
        self.residual_gain = kr_gain
        self.stiffness_inv = 1.0 / stiffness # Maps torque to position offset
        
        self.q_v = 0.0
        self.dq_v = 0.0

    def set_state(self, q_actual, dq_actual=0.0):
        self.q_v = q_actual
        self.dq_v = dq_actual

    def update(self, q_actual, r_vector, gravity_torque, dt):
        
        # Calculate the 'Reflex' component
        tau_reflex = self.residual_gain * r_vector
        
        error = self.q_v - q_actual
        spring_force = self.virtual_stiffness * error
        
        # Physics Integration (M*ddq + B*dq = tau_reflex)
        ddq = (tau_reflex + spring_force - (self.virtual_dumping * self.dq_v)) / self.virtual_mass
        self.dq_v += ddq * dt
        self.q_v += self.dq_v * dt
        
        # Apply Gravity Offset
        # Since we are position controlled, we translate the required 
        # gravity torque into a small position 'lead'
        q_gravity_offset = gravity_torque * self.stiffness_inv
        
        # The final command is the reflex motion + gravity offset
        q_cmd = self.q_v + q_gravity_offset
        
        return q_cmd
