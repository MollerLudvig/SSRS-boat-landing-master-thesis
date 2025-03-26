import numpy as np

class KalmanFilter2D:
    def __init__(self, dt, process_noise_variance=0.001):
        """
        Initialize the 2D Kalman filter.
        
        Parameters:
            dt: float
                The time step (delta t) for prediction (e.g. 0.1 for 10 Hz).
            process_noise_variance: float
                Scalar to scale the process noise covariance.
        """
        self.n = 4  # state dimension: [x, y, vx, vy]
        self.dt = dt

        # State transition matrix for constant velocity model
        self.A = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1,  0],
                           [0, 0, 0,  1]])
        
        # Initialize state: column vector [x, y, vx, vy]^T.
        # (For example, start at a given position with zero velocity.)
        self.x = np.zeros((self.n, 1))
        
        # Initial covariance (assume some uncertainty in all states)
        self.P = np.eye(self.n)
        
        # Process noise covariance matrix.
        # Derived from a constant acceleration model.
        dt2 = dt**2
        dt3 = dt**3 / 3
        dt2_half = dt**2 / 2
        self.Q = process_noise_variance * np.array([[dt3, 0,   dt2_half, 0],
                                                    [0,   dt3, 0,   dt2_half],
                                                    [dt2_half, 0,   dt,   0],
                                                    [0,   dt2_half, 0,   dt]])
        
        # Measurement matrix for camera: camera provides [x, y]
        self.H_camera = np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0]])
        
        # Measurement matrix for AIS: we assume AIS gives [x, y, vx, vy] (after conversion).
        self.H_AIS = np.eye(4)
        
    def predict(self):
        """Perform a prediction step using the constant velocity model."""
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q
        
    def update(self, z, H, R):
        """
        General update step of the Kalman filter.
        
        Parameters:
            z: np.array
                Measurement vector as a column vector.
            H: np.array
                Measurement matrix.
            R: np.array
                Measurement noise covariance matrix.
        """
        # Innovation (measurement residual)
        y = z - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Updated state estimate
        self.x = self.x + K @ y
        
        # Updated covariance estimate
        I = np.eye(self.n)
        self.P = (I - K @ H) @ self.P
        
    def update_camera(self, z, R_camera=None):
        """
        Update using a camera measurement which provides [x, y].
        
        Parameters:
            z: np.array
                Measurement vector of shape (2, 1).
            R_camera: np.array, optional
                2x2 measurement noise covariance matrix for the camera.
                Default is 0.01 * I.
        """
        if R_camera is None:
            R_camera = np.eye(2) * 0.01
        self.update(z, self.H_camera, R_camera)
        
    def update_AIS(self, z, R_AIS=None):
        """
        Update using AIS measurement. This function assumes AIS provides 
        measurements in the form [x, y, speed, yaw] if not already converted.
        If you receive speed and yaw, convert them to velocity components:
            vx = speed * cos(yaw)
            vy = speed * sin(yaw)
        Then, form a measurement vector [x, y, vx, vy]^T.
        
        Parameters:
            z: np.array
                Measurement vector of shape (4, 1) representing [x, y, vx, vy].
            R_AIS: np.array, optional
                4x4 measurement noise covariance matrix for AIS.
                Default is 0.01 * I.
        """
        if R_AIS is None:
            R_AIS = np.eye(4) * 0.01
        self.update(z, self.H_AIS, R_AIS)

# Example of how you might use the filter:

if __name__ == '__main__':
    # Initialize filter for 10 Hz predictions (dt = 0.1 s)
    kf = KalmanFilter2D(dt=0.1)
    
    # Suppose we start with an initial state at (x=100, y=50) and zero velocity.
    kf.x = np.array([[100],
                     [50],
                     [0],
                     [0]])
    
    # Simulate regular predictions at 10 Hz
    for t in range(50):  # simulate for 5 seconds
        kf.predict()
        # Here, you might normally use the predicted state for further processing.
        print("Prediction step:", kf.x.T)
        
        # If a camera measurement arrives (irregularly), call update_camera:
        if t % 15 == 0:  # for example, every 1.5 seconds
            # Suppose the camera measurement is (x=predicted+noise, y=predicted+noise)
            cam_meas = kf.x[0:2] + np.random.randn(2, 1) * 0.1
            kf.update_camera(cam_meas)
            print("Camera update:", kf.x.T)
            
        # Similarly, if AIS data arrives (irregularly), call update_AIS:
        if t % 20 == 0:  # for example, every 2 seconds
            # For AIS, assume we obtain x, y, speed, yaw.
            # Here we simulate a speed and yaw, then convert to velocity components.
            speed = 5 + np.random.randn()*0.1
            yaw = 0.1 + np.random.randn()*0.01  # in radians
            vx = speed * np.cos(yaw)
            vy = speed * np.sin(yaw)
            ais_meas = np.array([[kf.x[0,0] + np.random.randn()*0.1],  # x with noise
                                  [kf.x[1,0] + np.random.randn()*0.1],  # y with noise
                                  [vx],
                                  [vy]])
            kf.update_AIS(ais_meas)
            print("AIS update:", kf.x.T)
