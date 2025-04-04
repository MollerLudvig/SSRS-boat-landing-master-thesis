import numpy as np
import time
from collections import deque
import bisect

class KalmanFilterXY:
    def __init__(self, vx = 0, vy = 0, psi = 0, process_noise_variance=0.001, state_buffer_size=200, measurment_buffer_size=20, timestamp=time.time()):

        self.x = np.array([[0], [0], [vx], [vy], [psi]])  # State: [x, y, xdot, ydot, yaw]
        self.n = self.x.shape[0]
        self.P = np.eye(self.n)  # Covariance
        self.process_noise_variance = process_noise_variance
        self.last_time = timestamp
        self.state_buffer = deque(maxlen=state_buffer_size)   # Stores (timestamp, state, covariance) tuples
        self.state_buffer.append((self.last_time, self.x.copy(), self.P.copy()))  # Initialize buffer with the initial state
        self.measurement_buffer = deque(maxlen=measurment_buffer_size)  # Stores measurements

        # Measurement matrices
        self.H_camera = np.array([[1, 0, 0, 0, 0, 0],
                                  [0, 1, 0, 0, 0, 0]])
        
        self.H_AIS = np.array([[1, 0, 0, 0, 0], # x
                               [0, 1, 0, 0, 0], # y
                               [0, 0, 1, 0, 0], # xdot
                               [0, 0, 0, 1, 0], # ydot
                               [0, 0, 0, 0, 1],])# yaw
        
        
        # State transition matrix (updated in predict)
        self.F = np.eye(self.n)

    def predict(self, timestamp, past_timestamp=None):
        """Predict the state using a time-varying step."""
        last_measurment = self.measurement_buffer[-1] if self.measurement_buffer else None
        
        if past_timestamp is not None:
            dt = timestamp - past_timestamp
        else:
            dt = timestamp - self.last_time
            if dt <= 0.001:  # Ignore very small time steps
                return
            self.last_time = timestamp

        # print(f'dt: {dt}, timestamp: {timestamp}, past_timestamp: {past_timestamp}')
        # Update the state transition matrix F
        self.F = np.array([
            [1, 0, dt, 0, 0],
            [0, 1, 0, dt, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        # Predict state
        self.x = self.F @ self.x
    
        # Dynamic process noise covariance that scales with velocity
        v_scaler = 5
        dt2 = dt ** 2
        dt3 = dt ** 3 / 2
        dt4 = dt ** 4 / 4
        # Q = self.process_noise_variance * np.array([
        #     [dt*100, 0,0, 0, 0,],
        #     [0, dt*100, 0, 0, 0],
        #     [0, 0, dt3, 0, 0],
        #     [0, 0, 0, dt4, dt4],
        #     [0, 0, 0, dt2, dt]
        # ]) * (1 + np.abs(self.x[2,0]) * v_scaler)
        Q = self.process_noise_variance * np.array([
            [10, 0, 100, 0, 0],
            [0, 10, 0, 100, 0],
            [0, 0, 100, 0, 0],
            [0, 0, 0, 100, 0],
            [0, 0, 0, 0, 100]
        ]) * (1 + np.abs(self.x[2,0]) * v_scaler)

        # Update state covariance
        self.P = self.F @ self.P @ self.F.T + Q

        # Store state in buffer
        self.state_buffer.append((self.last_time, self.x.copy(), self.P.copy()))

    def update(self, z, H, R, timestamp):
        """Update the filter with a new measurement at a given timestamp."""
        if timestamp < self.last_time:  # Out-of-sequence measurement detected
            self._handle_OOSM(z, H, R, timestamp)
            return

        y = z - H @ self.x  # Innovation
        S = H @ (self.P @ H.T) + R  # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman Gain
        self.x = self.x + K @ y # Update state
        I = np.eye(self.n) # Identity matrix
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T  # Joseph form

    def _handle_OOSM(self, z, H, R, timestamp):
        """
        Handles out-of-sequence measurements by rolling back the state,
        applying the correction, and re-propagating to the current time.
        """
        # Step 1: Find the latest state before `timestamp`
        past_state = None
        for t, x, P in reversed(self.state_buffer):
            if t <= timestamp:
                past_state = (t, x.copy(), P.copy())
                break

        # If no past state is found, use the oldest available state
        if past_state is None:
            print("Warning: No valid past state found for OOSM! Using the oldest available state.")
            if len(self.state_buffer) > 0:
                past_state = self.state_buffer[0]  # Use oldest state
            else:
                print("Error: No states available in buffer. Ignoring OOSM.")
                return

        # Roll back to past state
        t_past, self.x, self.P = past_state


        # Step 2: Collect all later measurements (including the new OOSM)
        relevant_measurements = []
        for t, mz, mH, mR in reversed(self.measurement_buffer):
            if t < timestamp:
                break
            relevant_measurements.append((t, mz.copy(), mH.copy(), mR.copy()))
        relevant_measurements.append((timestamp, z.copy(), H.copy(), R.copy()))


        # Step 3: Apply measurements in chronological order
        for t, mz, mH, mR in reversed(relevant_measurements):
            self.predict(t, t_past)
            self.update(mz, mH, mR, t)
            t_past = t

        # Step 4: Predict to the current time
        self.predict(self.last_time, t_past)

    def _insert_measurement(self, z, H, R, timestamp):
        """Insert measurement into buffer while maintaining chronological order."""
        new_entry = (timestamp, z.copy(), H.copy(), R.copy())

        # Convert deque to list, insert in sorted order, and convert back to deque
        temp_list = list(self.measurement_buffer)
        bisect.insort(temp_list, new_entry, key=lambda x: x[0])
        self.measurement_buffer = deque(temp_list, maxlen=self.measurement_buffer.maxlen)

    def update_camera(self, z, timestamp, R_camera=None):
        """Update with a camera measurement ([x, y])."""
        if R_camera is None:
            R_camera = np.eye(2) * 0.01
        self._insert_measurement(z, self.H_camera, R_camera, timestamp)
        self.update(z, self.H_camera, R_camera, timestamp)

    def update_AIS(self, z, timestamp, R_AIS=None):
        """Update with AIS measurement ([x, y, speed, yaw])."""
        if R_AIS is None:
            # R_AIS = np.eye(4) * 0.01
            R_AIS = np.array([[10, 0, 0, 0, 0],
                              [0, 10, 0, 0, 0],
                              [0, 0, 1, 0, 0],
                              [0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 1]]) * 1
        self._insert_measurement(z, self.H_AIS, R_AIS, timestamp)
        self.update(z, self.H_AIS, R_AIS, timestamp)