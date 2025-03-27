import numpy as np
import time
from collections import deque

class KalmanFilter2DYawVariableDT:
    def __init__(self, process_noise_variance=0.001, buffer_size=10):
        self.n = 4  # State dimension: [x, y, v, yaw]
        self.x = np.zeros((self.n, 1))  # Initial state
        self.P = np.eye(self.n)  # Covariance
        self.process_noise_variance = process_noise_variance
        self.last_time = time.time()  # Time of last update
        self.state_buffer = deque(maxlen=buffer_size)  # Stores (timestamp, state, covariance)

        self.H_camera = np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0]])
        self.H_AIS = np.eye(4)

    def _update_dt(self):
        """Update dt based on the elapsed time since the last update."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        return dt

    def predict(self, dt=None):
        """Predict the state using a time-varying step."""
        if dt is None:
            dt = self._update_dt()

        x, y, v, psi = self.x.flatten()
        x_new = x + v * np.cos(psi) * dt
        y_new = y + v * np.sin(psi) * dt
        v_new = v
        psi_new = psi  # Assuming yaw is slowly changing

        self.x = np.array([[x_new], [y_new], [v_new], [psi_new]])

        # Compute linearized state transition matrix (F)
        F = np.array([
            [1, 0, np.cos(psi) * dt, -v * np.sin(psi) * dt],
            [0, 1, np.sin(psi) * dt,  v * np.cos(psi) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Process noise covariance (Q)
        dt2 = dt ** 2
        Q = self.process_noise_variance * np.array([
            [dt2, 0, 0, 0],
            [0, dt2, 0, 0],
            [0, 0, dt2, 0],
            [0, 0, 0, dt2]
        ])

        self.P = F @ self.P @ F.T + Q

        # Store state in buffer
        self.state_buffer.append((self.last_time, self.x.copy(), self.P.copy()))

    def update(self, z, H, R, timestamp):
        """Update the filter with a new measurement at a given timestamp."""
        if timestamp < self.last_time:  # Out-of-sequence measurement detected
            self.handle_OOSM(z, H, R, timestamp)
            return

        y = z - H @ self.x  # Innovation
        S = H @ self.P @ H.T + R  # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman Gain
        self.x = self.x + K @ y
        I = np.eye(self.n)
        self.P = (I - K @ H) @ self.P

    def handle_OOSM(self, z, H, R, timestamp):
        """
        Handles out-of-sequence measurements by rolling back the state,
        applying the correction, and re-propagating to the current time.
        """
        # Find the closest past state before the timestamp
        past_state = None
        for t, x, P in reversed(self.state_buffer):
            if t <= timestamp:
                past_state = (t, x.copy(), P.copy())
                break

        if past_state is None:
            print("Warning: No valid past state found for OOSM!")
            return

        t_past, x_past, P_past = past_state
        self.x, self.P = x_past, P_past  # Roll back

        # Compute dt from past time to measurement time
        dt_to_meas = timestamp - t_past

        # Predict from past state to measurement time
        self.predict(dt_to_meas)

        # Apply measurement update
        self.update(z, H, R, timestamp)

        # Re-propagate forward to the present
        dt_to_present = self.last_time - timestamp
        self.predict(dt_to_present)

    def update_camera(self, z, timestamp, R_camera=None):
        """Update with a camera measurement (global coordinates [x, y])."""
        if R_camera is None:
            R_camera = np.eye(2) * 0.01
        self.update(z, self.H_camera, R_camera, timestamp)

    def update_AIS(self, z, timestamp, R_AIS=None):
        """Update with AIS measurement (global [x, y, speed, yaw])."""
        if R_AIS is None:
            R_AIS = np.eye(4) * 0.01
        self.update(z, self.H_AIS, R_AIS, timestamp)


# Example usage:
if __name__ == '__main__':
    kf = KalmanFilter2DYawVariableDT()

    kf.x = np.array([[100], [50], [5], [0.1]])
    kf.last_time = time.time()

    import random

    for _ in range(20):
        current_time = time.time()
        kf.predict()
        print("Predicted state:", kf.x.T)

        if random.random() < 0.3:
            cam_meas = kf.x[0:2] + np.random.randn(2, 1) * 0.1
            kf.update_camera(cam_meas, current_time)
            print("Camera update:", kf.x.T)

        if random.random() < 0.2:
            delay = random.uniform(0, 1.5)  # Introduce a delay in AIS measurements
            ais_time = current_time - delay
            speed = kf.x[2, 0] + np.random.randn() * 0.1
            yaw = kf.x[3, 0] + np.random.randn() * 0.01
            ais_meas = np.array([[kf.x[0, 0] + np.random.randn() * 0.1],
                                 [kf.x[1, 0] + np.random.randn() * 0.1],
                                 [speed],
                                 [yaw]])
            kf.update_AIS(ais_meas, ais_time)
            print("AIS update:", kf.x.T)

        time.sleep(random.uniform(0.05, 0.2))
