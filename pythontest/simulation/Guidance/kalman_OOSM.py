import numpy as np
import time
from collections import deque
import bisect
# from Guidance.coordinate_conv import latlon_to_xy, xy_to_latlon, ned_to_latlon, latlon_to_ned
from coordinate_conv import latlon_to_xy, xy_to_latlon, ned_to_latlon, latlon_to_ned

verbose = False

class KalmanFilterXY:
    def __init__(self, u = 0, v = 0, heading = 0, init_lat = None, init_lon = None, process_noise_variance=0.001, state_buffer_size=200, measurment_buffer_size=20, timestamp=time.time()):

        heading_rad = np.deg2rad(heading)

        self.x = np.array([[0], [0], [heading_rad], [u], [v], [0]])  # State: [x, y, yaw (psi), vx (u), vy (v), yaw rate (r)]
        self.init_lat = init_lat
        self.init_lon = init_lon
        self.lat = None
        self.lon = None
        self.n = self.x.shape[0]
        self.P = np.eye(self.n)  # Covariance
        self.process_noise_variance = process_noise_variance
        self.last_time = timestamp
        self.state_buffer = deque(maxlen=state_buffer_size)   # Stores (timestamp, state, covariance) tuples
        self.state_buffer.append((self.last_time, self.x.copy(), self.P.copy()))  # Initialize buffer with the initial state
        self.measurement_buffer = deque(maxlen=measurment_buffer_size)  # Stores measurements

        # Measurement matrices
        self.H_camera = np.array([[1, 0, 0, 0, 0, 0], # x
                                  [0, 1, 0, 0, 0, 0]]) # y
        
        self.H_AIS = np.array([[1, 0, 0, 0, 0, 0], # x
                               [0, 1, 0, 0, 0, 0], # y
                               [0, 0, 1, 0, 0, 0], # yaw
                               [0, 0, 0, 1, 0, 0], # vx
                               [0, 0, 0, 0, 1, 0] # vy
                               ])
        
        # State transition matrix (updated in predict)
        self.F = np.eye(self.n)

    def _create_Q(self, dt, sigmaA, sigmaAR):
        """
        6×6 process noise covariance Q for state
        
        dt : float
            Time step Δt in seconds.
        sigmaA : float
            Standard deviation of linear acceleration (m/s²).
        sigmaAR : float
            Standard deviation of yaw acceleration (rad/s²).
        """
        # 2×2 block for position–velocity (x–u and y–v)
        q11 = (dt**4) / 4.0
        q12 = (dt**3) / 2.0
        q22 = (dt**2)
        q_pv = sigmaA**2 * np.array([[q11, q12],
                                    [q12, q22]])
        
        # 2×2 block for heading–yaw rate (psi–r)
        q_psi = sigmaAR**2 * np.array([[q11, q12],
                                    [q12, q22]])
        
        # Assemble 6×6 block‑diagonal Q
        Q = np.zeros((6, 6))
        
        # indices: x=0, y=1, psi=2, u=3, v=4, r=5
        # insert x–u block
        Q[np.ix_([0, 3], [0, 3])] = q_pv
        # insert y–v block
        Q[np.ix_([1, 4], [1, 4])] = q_pv
        # insert psi–r block
        Q[np.ix_([2, 5], [2, 5])] = q_psi
    
        return Q

    def predict(self, timestamp, past_timestamp=None):
        """Predict the state using a time-varying step."""
        last_measurment = self.measurement_buffer[-1] if self.measurement_buffer else None
        
        if past_timestamp is not None:
            dt = timestamp - past_timestamp
        else:
            dt = timestamp - self.last_time
            if dt <= 0.01:  # Ignore very small time steps
                if verbose:
                    print(f"dt too small: {dt}")
                return


        if verbose:
            print("\n")
            print(f"dt: {dt}")
            print(f"Heading: {self.x[3,0]}")
            print(f"sepeed: {self.x[2,0]}")
            print("\n")

        # Update the state transition matrix F
        psi = self.x[2, 0]

        self.F = np.array([
            [1, 0, 0, np.cos(psi) * dt, -np.sin(psi) * dt, 0],
            [0, 1, 0, np.sin(psi) * dt, np.cos(psi) * dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # F = np.array([
        #     [1, 0, 0, dt, 0, 0],
        #     [0, 1, 0, 0, dt, 0],
        #     [0, 0, 1, 0, 0, dt],
        #     [0, 0, 0, 1, 0, 0],
        #     [0, 0, 0, 0, 1, 0],
        #     [0, 0, 0, 0, 0, 1]
        # ])

        # Predict state
        x_old = self.x # For prints
        self.x = self.F @ self.x

        # Set lat lon
        self.lat, self.lon = ned_to_latlon(self.x[0][0], self.x[1][0], self.init_lat, self.init_lon)
    
        # Process noise covariance
        sigmaA = 0.2          # m/s²
        sigmaAR = 0.02        # rad/s²
        Q = self._create_Q(dt, sigmaA, sigmaAR)

        # Update state covariance
        self.P = self.F @ self.P @ self.F.T + Q

        # Store state in buffer
        self.state_buffer.append((timestamp, self.x.copy(), self.P.copy()))

        # Print debug information
        if verbose:
            print("\n")
            print("Predict step:")
            print(f"dt: {dt}, last_time: {self.last_time}, timestamp: {timestamp}")
            print(f"x: {self.x[0][0]}, y: {self.x[1][0]}, psi: {self.x[2][0]}, u: {self.x[3][0]}, v: {self.x[4][0]}")
            print(f"lat: {self.lat}, lon: {self.lon}")
            print("\n")

        # Update last time
        self.last_time = timestamp



    def update(self, z, H, R, timestamp):
        """
        Update the filter with a new measurement at a given timestamp.
        Handles regular updates and delegates OOSM processing when needed.
        """
        # First, store the measurement in the buffer
        self._insert_measurement(z, H, R, timestamp)
        
        # If the measurement is at the current time or in the future, do a regular update
        if timestamp >= self.last_time:
            # Predict to the measurement time if needed
            if timestamp > self.last_time:
                self.predict_EKF(timestamp)
            
            # Calculate innovation
            y = z - H @ self.x
            
            # Handle angle wrapping if needed
            if y.shape[0] > 2 and H.shape[0] > 2:
                y[2][0] = self.wrap_angle_rad(y[2][0])
            
            # Standard Kalman filter update
            S = H @ (self.P @ H.T) + R  # Innovation covariance
            K = self.P @ H.T @ np.linalg.inv(S)  # Kalman Gain
            self.x = self.x + K @ y # Update state
            I = np.eye(self.n) # Identity matrix
            self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T  # Joseph form


            # Update lat/lon
            self.lat, self.lon = ned_to_latlon(self.x[0][0], self.x[1][0], self.init_lat, self.init_lon)
            
            # Update the state buffer
            self.state_buffer.append((timestamp, self.x.copy(), self.P.copy()))
            self.last_time = timestamp
        else:
            # Handle out-of-sequence measurement
            self._handle_OOSM(z, H, R, timestamp)



    def predict_EKF(self, timestamp, past_timestamp=None):
        if past_timestamp is not None:
            dt = timestamp - past_timestamp
        else:
            dt = timestamp - self.last_time
            if dt <= 0.01:
                return

        # Predict state using nonlinear model
        self.x = self.transition_function(self.x, dt)

        # Compute Jacobian F
        F = self.compute_jacobian(self.x, dt)

        # Create process noise
        sigmaA = 0.2
        sigmaAR = 0.02
        Q = self._create_Q(dt, sigmaA, sigmaAR)

        # Predict covariance using linearized dynamics
        self.P = F @ self.P @ F.T + Q

        # Update lat/lon
        self.lat, self.lon = ned_to_latlon(self.x[0, 0], self.x[1, 0], self.init_lat, self.init_lon)

        # Save and update time
        self.state_buffer.append((timestamp, self.x.copy(), self.P.copy()))
        self.last_time = timestamp


    def transition_function(self, x, dt):
        """Applies nonlinear motion model based on current state."""
        x_pos, y_pos, psi, u, v, r = x.flatten()

        dx = np.array([
            [x_pos + (u * np.cos(psi) - v * np.sin(psi)) * dt],
            [y_pos + (u * np.sin(psi) + v * np.cos(psi)) * dt],
            [psi + r * dt],
            [u],
            [v],
            [r]
        ])
        # Wrap angles to [-pi, pi]
        dx[2][0] = self.wrap_angle_rad(dx[2][0])

        return dx


    def compute_jacobian(self, x, dt):
        """Computes Jacobian of the transition function wrt state."""
        _, _, psi, u, v, _ = x.flatten()

        F = np.eye(6)
        F[0, 2] = (-u * np.sin(psi) - v * np.cos(psi)) * dt
        F[0, 3] =  np.cos(psi) * dt
        F[0, 4] = -np.sin(psi) * dt

        F[1, 2] = ( u * np.cos(psi) - v * np.sin(psi)) * dt
        F[1, 3] =  np.sin(psi) * dt
        F[1, 4] =  np.cos(psi) * dt

        F[2, 5] = dt  # yaw += r * dt

        return F

    def _handle_OOSM(self, z, H, R, timestamp):
        """
        Handles out-of-sequence measurements by rolling back the state,
        applying the correction, and re-propagating to the current time.
        """
        # Store the current time as we'll need to propagate back to it
        current_time = self.last_time
        
        # Step 1: Find the latest state before the OOSM timestamp
        past_state = None
        past_time = None
        for t, x, P in reversed(self.state_buffer):
            if t <= timestamp:
                past_state = (t, x.copy(), P.copy())
                past_time = t
                break

        # If no valid past state found, use the oldest available state with a warning
        if past_state is None:
            if len(self.state_buffer) > 0:
                print("Warning: No state found before OOSM timestamp. Using oldest state.")
                past_state = self.state_buffer[0]
                past_time = past_state[0]
            else:
                print("Error: No states available in buffer. Ignoring OOSM.")
                return

        # Step 2: Insert the new measurement into the buffer to maintain chronological order
        self._insert_measurement(z, H, R, timestamp)

        # Step 3: Rollback to the past state
        self.x = past_state[1]
        self.P = past_state[2]
        self.last_time = past_time

        # Step 4: Collect all measurements from past_time to current_time in chronological order
        future_measurements = []
        for item in self.measurement_buffer:
            t, mz, mH, mR = item
            if past_time < t <= current_time:
                future_measurements.append(item)

        # Ensure measurements are sorted by timestamp
        future_measurements.sort(key=lambda x: x[0])

        # Step 5: Process all measurements sequentially
        last_processed_time = past_time
        
        for t, mz, mH, mR in future_measurements:
            # Predict to the measurement time
            if t > last_processed_time:
                self.predict_EKF(t, last_processed_time)

            # Update with the measurement
            self.update(mz, mH, mR, t)

        # Update the state buffer with this reprocessed state
        self.state_buffer.append((t, self.x.copy(), self.P.copy()))
        last_processed_time = t


        # Step 6: If current_time is ahead of last measurement time, predict to current_time
        if current_time > last_processed_time:
            self.predict_EKF(current_time, last_processed_time)


        # Step 7: Clean up state buffer to maintain chronological order
        # This removes any states that were added out-of-sequence during reprocessing
        temp_list = list(self.state_buffer)
        temp_list.sort(key=lambda x: x[0])
        self.state_buffer = deque(temp_list, maxlen=self.state_buffer.maxlen)
        
        # Step 8: Update the last time
        self.last_time = current_time




    def _insert_measurement(self, z, H, R, timestamp):
        """
        Insert measurement into buffer while maintaining chronological order.
        Returns True if the measurement was added, False if it was too old.
        """
        # Check if the measurement is too old (older than our oldest state)
        if self.state_buffer and timestamp < self.state_buffer[0][0]:
            print(f"Warning: Measurement at {timestamp} is older than oldest state " +
                f"({self.state_buffer[0][0]}). Discarding.")
            return False
        
        # Create new measurement entry
        new_entry = (timestamp, z.copy(), H.copy(), R.copy())
        
        # Insert into buffer maintaining chronological order
        # Convert deque to list for easier manipulation
        temp_list = list(self.measurement_buffer)
        
        # Find insertion point
        insert_idx = 0
        for i, (t, _, _, _) in enumerate(temp_list):
            if t > timestamp:
                insert_idx = i
                break
            insert_idx = i + 1
        
        # Insert the measurement
        temp_list.insert(insert_idx, new_entry)
        
        # Convert back to deque with limited length
        self.measurement_buffer = deque(temp_list, maxlen=self.measurement_buffer.maxlen)
        return True

    def _local_to_global(self, delta_x, delta_y, measurement_point_lat, measurement_point_lon, measurement_point_heading):
        """Convert local offset (delta_x, delta_y) to a global lat/lon position,
        given a reference point in lat/lon and a compass heading in degrees.
        delta_x is transversal distance to the right, delta_y is longitudinal distance.
        """
        # Get the reference point in global xy
        x, y = latlon_to_ned(measurement_point_lat, measurement_point_lon, self.init_lat, self.init_lon)

        # TODO: these cos sin are probably not correct anymore
        # PROBABLY GOOD NOW I THINK

        # Convert local offset to global NED coordinates
        # For North=0°, East=90° convention:
        dx_global = delta_x * np.cos(measurement_point_heading) - delta_y * np.sin(measurement_point_heading)
        dy_global = delta_x * np.sin(measurement_point_heading) + delta_y * np.cos(measurement_point_heading)

        # Apply offset
        global_x = x + dx_global
        global_y = y + dy_global

        return global_x, global_y


    def update_camera(self, z, drone_lat, drone_lon, drone_heading, timestamp, R_camera=None):
        """Update with a camera measurement (z = [[x], [y]])."""
        if R_camera is None:
            R_camera = np.eye(2) * 0.01

        drone_heading_rad = np.deg2rad(drone_heading)

        # Convert from an offset measurement to a global xy position
        z[0][0], z[1][0] = self._local_to_global(z[0][0], z[1][0], drone_lat, drone_lon, drone_heading_rad)
        
        # Update filter
        self.update(z, self.H_camera, R_camera, timestamp)

    def update_AIS(self, z, timestamp, R_AIS=None):
        """Update with AIS measurement z = [[x], [y], [speed], [heading (degrees)]."""
        if R_AIS is None:
            R_AIS = np.eye(5) * 0.001
            R_AIS[0][0] = 0.001  # x
            R_AIS[1][1] = 0.001  # y
            R_AIS[2][2] = 0.001  # heading
            R_AIS[3][3] = 0.01  # u
            R_AIS[4][4] = 0.01  # v
            # R_AIS = np.array([[10, 0, 0, 0],
            #                   [0, 10, 0, 0],
            #                   [0, 0, 10, 0],
            #                   [0, 0, 0, 1]]) * 0.001

        # Convert heading to radians
        z[2][0] = np.deg2rad(z[2][0])

        self.update(z, self.H_AIS, R_AIS, timestamp)

    def update_w_latlon(self, z, timestamp, R_AIS=None):
        """ Assume z = [[lat], [lon], [heading (degrees)], [u], [v]]"""

        # Substitute lat lon with x, y in measurment vector
        # z[0][0], z[1][0] = latlon_to_xy(z[0][0], z[1][0], self.init_lat, self.init_lon)

        z[0][0], z[1][0] = latlon_to_ned(z[0][0], z[1][0], self.init_lat, self.init_lon)

        # Update through conventional function
        self.update_AIS(z,timestamp, R_AIS)


    def wrap_angle_rad(self, angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

