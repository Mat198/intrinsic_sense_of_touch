import numpy as np
import time

np.set_printoptions(precision=4, suppress=True)

class CollisionDetector:
    def __init__(self, gain, threshold, joint_number):
        self.gain = np.diag([gain] * joint_number)
        self.residual_threshold = threshold
        self.last_state_stamp = None
        self.residuals = np.array([0.0] * joint_number)
        self.residual_integral = 0
        self.last_print_stamp = 0.0
        
    def calculate_residual_torque(self, residual, momentum, alpha_model, efforts, current_time):
        """
        Calculates the residual torque observer: r = K integral(alpha - tau - r)dt + p

        :param residual: list or np.array of current residuals
        :param momentum: list or np.array of current generalized momentum
        :param alpha_model: list or np.array of g(q) - Ct(q,q_dot)
        :param efforts: list or np.array of current joint torques (tau)
        :param current_time: float (seconds) or an object with a .seconds() method

        :return: np.array of updated residuals
        """
        
        if residual.size != efforts.size:
            print("Failed to calculate residual! Efforts and residual have different shapes!")
            return np.array([])

        # Handle time delta (dt)
        if self.last_state_stamp is None:
            dt = 0.0
        else:
            # Assuming current_time is a float or has a way to get seconds
            dt = current_time - self.last_state_stamp
            
        self.last_state_stamp = current_time

        # Calculate the residual integral
        self.residual_integral += (-efforts - residual + alpha_model) * dt

        # Compute Residual
        new_residual = self.gain @ self.residual_integral + momentum

        return new_residual

    def identify_collision(self, momentum, alpha_model, efforts, current_time):

        efforts = np.array(efforts)

        self.residuals = self.calculate_residual_torque(
            self.residuals, momentum, alpha_model, efforts, current_time)
        
        in_collision = np.abs(self.residuals) > self.residual_threshold
        for i in range(len(in_collision) -1, 1, -1):
            if in_collision[i-1] and not in_collision[i]:
                if (current_time - self.last_print_stamp) > 2.0:
                    print(f"Link {i-1} is in collision state!")
                    print(f"Efforts:   {efforts}")
                    print(f"Residuals: {self.residuals}")
                    self.last_print_stamp = current_time
        return in_collision.tolist(), self.residuals