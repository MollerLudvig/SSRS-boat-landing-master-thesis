import numpy as np
import time
from collections import deque
# from Guidance.coordinate_conv import latlon_to_xy, xy_to_latlon, ned_to_latlon, latlon_to_ned
from coordinate_conv import latlon_to_xy, xy_to_latlon, ned_to_latlon, latlon_to_ned

verbose = False

class KalmanFilterXY:
    def __init__(self, u = 0, v = 0, heading = 0, init_lat = None, init_lon = None, process_noise_variance=1, state_buffer_size=200, measurment_buffer_size=20, timestamp=time.time()):

        heading_rad = np.deg2rad(heading)

        self.x = np.array([[0], [0], [heading_rad], [u], [v], [0], [0], [0]])  # State: [x, y, yaw (psi), vx (u), vy (v), yaw rate (r), ax, ay]
        self.init_lat = init_lat
        self.init_lon = init_lon
        self.lat = None
        self.lon = None
        self.n = self.x.shape[0]
        self.P = np.eye(self.n)  # Covariance
        self.P *= process_noise_variance  # Scale covariance by process noise variance
        self.last_time = timestamp
        self.state_buffer = deque(maxlen=state_buffer_size)   # Stores (timestamp, state, covariance) tuples
        self.state_buffer.append((self.last_time, self.x.copy(), self.P.copy()))  # Initialize buffer with the initial state
        self.measurement_buffer = deque(maxlen=measurment_buffer_size)  # Stores measurements
    


    def _create_Q(self, dt, sigmaA, sigmaAR, sigmaJ):
        """
        Create 8×8 process noise covariance matrix Q.

        dt : float
            Time step Δt in seconds.
        sigmaA : float
            Standard deviation of linear acceleration (m/s²).
        sigmaAR : float
            Standard deviation of yaw acceleration (rad/s²).
        sigmaJ : float
            Standard deviation of jerk (m/s³).

        State: [x, y, psi, u, v, r, ax, ay]

        """
        Q = np.zeros((8, 8))

        # Process noise for x, y (via u, ax), and u, ax coupling
        q_posVel = sigmaA**2 * np.array([
            [(dt**4)/4, (dt**3)/2],
            [(dt**3)/2, (dt**2)]
        ])
        # For both x/u and y/v
        Q[np.ix_([0, 3], [0, 3])] = q_posVel
        Q[np.ix_([1, 4], [1, 4])] = q_posVel

        # Yaw + yaw rate
        q_yaw = sigmaAR**2 * np.array([
            [(dt**4)/4, (dt**3)/2],
            [(dt**3)/2, (dt**2)]
        ])
        Q[np.ix_([2, 5], [2, 5])] = q_yaw

        # Acceleration noise (modeled as random walk or constant jerk input)
        Q[6, 6] = sigmaJ**2 * dt  # ax
        Q[7, 7] = sigmaJ**2 * dt  # ay

        return Q


    def _update_EKF(self, z, R, timestamp, extra_states=None, skip_prediction=False):

        # if not skip_prediction:
        #     if timestamp < self.last_time:
        #         # Handle out-of-sequence measurement
        #         self._handle_OOSM(timestamp)
        #     else:   
        #         # Predict to the measurement time if needed
        #         if timestamp > self.last_time:
        #             self.predict_EKF(timestamp)
        #         else:
        #             # Check if the state is older than newest state
        #             # Remove all states newer than current timestamp
        #             # This also happens in predict but for "updates only" its still needed
        #             if self.state_buffer and timestamp <= self.state_buffer[-1][0]:
        #                 self.state_buffer = [item for item in self.state_buffer if item[0] < timestamp]

        if not skip_prediction:
            if timestamp < self.last_time:
                # Handle out-of-sequence measurement
                self._handle_OOSM(timestamp)
            elif timestamp > self.last_time:
                # Predict to the measurement time if needed
                self.predict_EKF(timestamp)


        # Get h and H based on the measurement type 
        if z.shape[0] == 2:
            # Camera measurement
            H = self._get_H_Camera(extra_states['drone_heading_rad'])
            h = self._get_h_Camera(extra_states['drone_heading_rad'], self.x[0][0], self.x[1][0], extra_states['drone_N'], extra_states['drone_E'])
        elif z.shape[0] == 5:
            # AIS measurement
            H = self._get_H_AIS()
            h = self._get_h_AIS()
        else:
            raise ValueError(f"Unsupported measurement shape: {z.shape}. Expected (2,1) for camera or (5,1) for AIS.")


        # Calculate innovation
        y = z - h
        y[2][0] = self.wrap_angle_rad(y[2][0])

        # Innovation covariance
        S = H @ (self.P @ H.T) + R 

        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y
        self.x[2][0] = self.wrap_angle_rad(self.x[2][0])
            
        # Joseph form for covariance update
        I = np.eye(self.n)  # Identity matrix
        self.P = (I - K @ H) @ self.P #@ (I - K @ H).T + K @ R @ K.T
           

        # Update lat/lon
        self.lat, self.lon = ned_to_latlon(self.x[0][0], self.x[1][0], self.init_lat, self.init_lon)
        # Update the state buffer and time
        self.state_buffer.append((timestamp, self.x.copy(), self.P.copy()))
        self.last_time = timestamp


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
        F = self.compute_jacobian(dt)

        # Create process noise covariance Q
        sigmaA = 0.5 # Acceleration influencing position
        sigmaAR = 0.05 # Angular acceleration (rad/s²)
        sigmaJ = 0.5     # Jerk magnitude (m/s³), if modeling ax/ay as slowly changing
        Q = self._create_Q(dt, sigmaA, sigmaAR, sigmaJ)
        # Q = self.compute_q()
        # Q = np.eye(self.n) * self.process_noise_variance

        # Ceck size of Q
        if Q.shape != (self.n, self.n):
            raise ValueError(f"Q matrix size mismatch: expected {self.n}x{self.n}, got {Q.shape[0]}x{Q.shape[1]}")

        # Predict covariance using linearized dynamics
        self.P = F @ self.P @ F.T + Q

        # Update lat/lon
        self.lat, self.lon = ned_to_latlon(self.x[0, 0], self.x[1, 0], self.init_lat, self.init_lon)


        # Save and update timestamp
        # Check if the state is older than newest state
        # Remove all states newer than current timestamp
        if self.state_buffer and timestamp <= self.state_buffer[-1][0]:
            self.state_buffer = [item for item in self.state_buffer if item[0] < timestamp]

        # Append the new state to the buffer
        self.state_buffer.append((timestamp, self.x.copy(), self.P.copy()))
        self.last_time = timestamp


    def transition_function(self, x, dt):
        """Applies nonlinear motion model based on current state."""
        x_pos, y_pos, psi, u, v, r, u_acc, v_acc = x.flatten()

        dx = np.array([
            [x_pos +
             (u * np.cos(psi) - v * np.sin(psi)) * dt +
             0.5 * (u_acc * np.cos(psi) - v_acc * np.sin(psi)) * dt**2
             ],
            [y_pos + 
             (u * np.sin(psi) + v * np.cos(psi)) * dt +
             0.5 * (u_acc * np.sin(psi) + v_acc * np.cos(psi)) * dt**2
             ],
            [psi + r * dt],
            [u + u_acc * dt],
            [v + v_acc * dt],
            [r],
            [u_acc],
            [v_acc]
        ])
        # Wrap angles to [-pi, pi]
        dx[2][0] = self.wrap_angle_rad(dx[2][0])

        return dx

    
    def compute_jacobian(self, dt):
        _, _, yaw, u, v, r, ax, ay = self.x.flatten()
        cosPsi = np.cos(yaw)
        sinPsi = np.sin(yaw)
        dt2 = dt ** 2 / 2

        F = np.eye(8)

        # Position x
        F[0, 2] = -u * sinPsi * dt - v * cosPsi * dt - ax * sinPsi * dt2 - ay * cosPsi * dt2
        F[0, 3] = cosPsi * dt
        F[0, 4] = -sinPsi * dt
        F[0, 6] = cosPsi * dt2
        F[0, 7] = -sinPsi * dt2

        # Position y
        F[1, 2] = u * cosPsi * dt - v * sinPsi * dt + ax * cosPsi * dt2 - ay * sinPsi * dt2
        F[1, 3] = sinPsi * dt
        F[1, 4] = cosPsi * dt
        F[1, 6] = sinPsi * dt2
        F[1, 7] = cosPsi * dt2

        # Yaw (psi)
        F[2, 5] = dt

        # Velocity u
        F[3, 6] = dt

        # Velocity v
        F[4, 7] = dt

        # Rest (r, ax, ay) are constant → already set by identity

        return F

    def _handle_OOSM(self, timestamp):
        """
        Handles out-of-sequence measurements by rolling back the state,
        applying the correction, and re-propagating to the current time.
        """
        # Store the current time. Its need to propagate back to it latre
        current_time = self.last_time
        
        # Step 1: Find the latest state before the OOSM timestamp
        # This is the state we will roll back to
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

        # Step 2: Rollback to the past state
        self.x = past_state[1]
        self.P = past_state[2]
        self.last_time = past_time

        # Step 3: Collect all measurements from past_time to current_time in chronological order
        future_measurements = []
        for item in self.measurement_buffer:
            t, mz, mR, mExtra = item
            if past_time < t <= current_time:
                future_measurements.append(item)

        # Sort future measurements by timestamp for good measure
        # future_measurements.sort(key=lambda x: x[0])

        # Step 4: Process all measurements sequentially
        for t, mz, mR, mExtra in future_measurements:
            self._update_EKF(mz, mR, t, mExtra, skip_prediction=True)        


    def _insert_measurement(self, z, R, timestamp, extra_states = None):
        """
        Insert measurement into buffer while maintaining chronological order.
        """
        # Create new measurement entry
        new_entry = (timestamp, z.copy(), R.copy(), extra_states.copy() if extra_states is not None else None)
        
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



    def update_camera(self, z, timestamp, drone_lat, drone_lon, drone_heading_deg, drone_alt, ship_alt = 0):
        """
        Update the filter with a camera measurement.
        z: np.array of shape (2, 1)
        - z[0]: North offset (in meters)
        - z[1]: East offset (in meters)
        drone_lat: Latitude of the drone (in degrees)
        drone_lon: Longitude of the drone (in degrees)
        drone_alt: Altitude of the drone (in meters)
        ship_alt: Altitude of the ship (in meters, default is 0)
        drone_heading_deg: Heading of the drone (or camera, whatever way the x is pointing) (in degrees)
        """
        drone_heading_rad = self.wrap_angle_rad(np.deg2rad(drone_heading_deg))

        # Get R_Camera if not provided
        R = self._get_R_Camera(z[0][0], z[1][0], drone_alt - ship_alt)

        # Convert local offset to global NED coordinates
        drone_N, drone_E = latlon_to_ned(drone_lat, drone_lon, self.init_lat, self.init_lon)
        extra_states = {
            'drone_heading_rad': drone_heading_rad,
            'drone_N': drone_N,
            'drone_E': drone_E
        }

        # Insert the measurement into the buffer
        self._insert_measurement(z, R, timestamp, extra_states)

        # Update the filter with the measurement
        self._update_EKF(z, R, timestamp, extra_states)



    # def update_AIS(self, lat, lon, heading_deg, course_deg, velocity, timestamp, R_AIS=None):
    def update_AIS(self, z, timestamp):
        """
        Update the filter with an AIS measurement.
        z: np.array of shape (5, 1)
        - z[0]: latitude
        - z[1]: longitude
        - z[2]: heading in degrees
        - z[3]: course in degrees
        - z[4]: velocity in m/s
        """
        # Prerpocess the AIS measurement 
        z[0][0], z[1][0] = latlon_to_ned(z[0][0], z[1][0], self.init_lat, self.init_lon)
        z[2][0] = self.wrap_angle_rad(np.deg2rad(z[2][0]))  # heading in radians
        z[3][0] = self.wrap_angle_rad(np.deg2rad(z[3][0]))  # course in radians

        # Get R_AIS if not provided
        R = self._get_R_AIS(z[4][0])

        # Insert the measurement into the buffer
        self._insert_measurement(z, R, timestamp)

        self._update_EKF(z, R, timestamp)

    def _get_h_AIS(self):
        """Get the measurement function for AIS measurements, with v always negated."""
        u = self.x[3,0]
        v = self.x[4,0]

        course = np.arctan2(v, u)
        speed  = np.hypot(u, v)

        h = np.zeros((5,1))
        h[0,0] = self.x[0,0]   # N
        h[1,0] = self.x[1,0]   # E
        h[2,0] = self.x[2,0]   # heading (psi)
        h[3,0] = self.x[2,0] + course  # course (psi + course)
        h[4,0] = speed
        return h
    

    def _get_H_AIS(self):
        """Get Jacobian of AIS measurement model h_AIS(x)."""
        u = self.x[3, 0]
        v = self.x[4, 0]

        denom = u**2 + v**2 + 1e-6  # Avoid divide-by-zero
        speed = np.sqrt(denom)

        H = np.zeros((5, self.n))

        # ∂N/∂x, ∂E/∂y, ∂ψ/∂ψ
        H[0, 0] = 1  # ∂/∂N
        H[1, 1] = 1  # ∂/∂E
        H[2, 2] = 1  # ∂/∂ψ

        # ∂ε/∂ψ, ∂ε/∂u, ∂ε/∂v
        # ε = ψ + atan2(v, u)
        H[3, 2] = 1
        H[3, 3] = -v / denom  # ∂/∂u of atan2(v, u)
        H[3, 4] = u / denom   # ∂/∂v of atan2(v, u)

        # ∂V/∂u, ∂V/∂v
        # V = sqrt(u^2 + v^2)
        H[4, 3] = u / speed
        H[4, 4] = v / speed

        return H

    # def _get_R_AIS(self):
    #     R_AIS = np.zeros((5,5))
    #     R_AIS[0][0] = 5  # x
    #     R_AIS[1][1] = 5  # y
    #     R_AIS[2][2] = 0.05  # heading
    #     R_AIS[3][3] = 0.05  # course
    #     R_AIS[4][4] = 0.1  # velocity
    #     return R_AIS
    def _get_R_AIS(self, V, sigmaGPS=5.0, dtErr=0.5, sigmaLow=0.05, sigmaHigh=1.0, V0=2.0, zeta=2.0, sigmaV=0.5):
        """
        Get the measurement noise covariance matrix for AIS measurements.

        Parameters:
            V : float
                AIS-reported speed over ground (used for velocity-dependent uncertainty).
            sigmaGPS : float
                Baseline GPS positional error (m).
            dtErr : float
                Timestamp rounding uncertainty (s).
            sigmaLow : float
                Minimum angular error (rad).
            sigmaHigh : float
                Maximum angular error (rad).
            V0 : float
                Midpoint velocity for sigmoid curve.
            zeta : float
                Slope of the sigmoid curve.
            sigmaV : float
                Standard deviation of velocity measurement (m/s).
        """
        sigmaXY = sigmaGPS + dtErr * V
        sigmaDeg = sigmaLow + (sigmaHigh - sigmaLow) / (1 + np.exp(-zeta * (V0 - V)))

        R_AIS = np.zeros((5, 5))
        R_AIS[0, 0] = sigmaXY ** 2       # N
        R_AIS[1, 1] = sigmaXY ** 2       # E
        R_AIS[2, 2] = sigmaDeg ** 2      # Heading (yaw)
        R_AIS[3, 3] = sigmaDeg ** 2      # Course
        R_AIS[4, 4] = sigmaV ** 2        # Velocity magnitude
        return R_AIS


    
    def _get_h_Camera(self, drone_heading_rad, boat_N, boat_E, drone_N, drone_E):
        """Get the measurement function for camera measurements."""
        
        est_N = (boat_N - drone_N) * np.cos(drone_heading_rad) + (boat_E - drone_E) * np.sin(drone_heading_rad)
        est_E = -(boat_N - drone_N) * np.sin(drone_heading_rad) + (boat_E - drone_E) * np.cos(drone_heading_rad)

        h = np.zeros((2, 1))
        h[0][0] = est_N
        h[1][0] = est_E

        return h

    def _get_H_Camera(self, drone_heading_rad):
        """Get jacobian of the measurement function for camera measurements."""
        H = np.zeros((2,self.n))
        
        H[0][0], H[1][1] = np.cos(drone_heading_rad)
        H[1][0] = -np.sin(drone_heading_rad)
        H[0][1] = np.sin(drone_heading_rad)

        return H
        
    # def _get_R_Camera(self):
    #     """Get the measurement noise covariance matrix for camera measurements."""
    #     R_camera = np.eye(2) * 0.01
    #     return R_camera
    def _get_R_Camera(self, deltaX, deltaY, deltaZ, sigmaX=0.05, sigmaAlpha=0.01):
        """
        Get the measurement noise covariance matrix for camera-based relative measurements.

        Parameters:
            deltaX, deltaY, deltaZ : float
                Relative position components from drone to vessel (in meters).
            sigmaX : float
                Scaling factor for forward distance error.
            sigmaAlpha : float
                Scaling factor for lateral angular error.
        """
        d = np.sqrt(deltaX**2 + deltaY**2 + deltaZ**2)

        R_camera = np.zeros((2, 2))
        R_camera[0, 0] = (sigmaX * d) ** 2       # Forward error
        R_camera[1, 1] = (sigmaAlpha * d) ** 2   # Lateral error
        return R_camera

    


    def wrap_angle_rad(self, angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

