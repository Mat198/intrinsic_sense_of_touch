import numpy as np

class CollisionDetector:
    def __init__(self, gain, threshold, joint_number):
        self.gain = np.diag([gain] * joint_number)
        self.residual_threshold = threshold
        self.last_state_stamp = None
        self.residuals = np.array([0.0] * joint_number)
        self.residual_integral = 0
        self.last_print_stamp = 0.0

    def reset(self):
        self.residual_integral = 0
        
    def calculate_residual_torque(self, residual, momentum, alpha_model, efforts, dt):
        # Calculates the residual torque observer: r = K [p - integral{(efforts + r - alpha) dt}] 
        
        if residual.size != efforts.size:
            print("Failed to calculate residual! Efforts and residual have different shapes!")
            return np.array([])

        self.residual_integral += (efforts + residual - alpha_model) * dt
        new_residual = self.gain @ (momentum - self.residual_integral) 
        return new_residual

    def identify_collision(self, momentum, alpha_model, efforts, dt):

        efforts = np.array(efforts)

        self.residuals = self.calculate_residual_torque(
            self.residuals, momentum, alpha_model, efforts, dt)
        
        in_collision = np.abs(self.residuals) > self.residual_threshold
        return in_collision.tolist(), self.residuals