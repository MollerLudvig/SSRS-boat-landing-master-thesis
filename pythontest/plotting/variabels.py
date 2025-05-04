from VehicleMonitor import VehicleMonitor

class Variables:
    """
    Jag vet att detta är jättedummt men VehicleMonitor är ej 
    tread safe om man tar variabler från objektet. 
    Här sparas variablert för enkel acess utanför den klassen.
    """
    def __init__(self, object):
        self.object = object

        self.pos_time = None
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None
        self.pos_vx = None
        self.pos_vy = None
        self.pos_vz = None

        self.att_time = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.roll_speed = None
        self.pitch_speed = None
        self.yaw_speed = None
        
        self.gps_time = None
        self.gps_lat = None
        self.gps_lon = None
        self.gps_alt = None
        self.gps_relative_alt = None
        self.gps_vx = None
        self.gps_vy = None
        self.gps_vz = None
        self.gps_hdg = None
        
        self.imu_time = None
        self.imu_ax = None
        self.imu_ay = None
        self.imu_az = None
        self.imu_wx = None
        self.imu_wy = None
        self.imu_wz = None

        self.wind_time = None
        self.wind_speed = None
        self.wind_direction = None

        self.sim_time = None
        self.sim_roll = None
        self.sim_pitch = None
        self.sim_yaw = None
        self.sim_xacc = None
        self.sim_yacc = None
        self.sim_zacc = None
        self.sim_xgyro = None
        self.sim_ygyro = None
        self.sim_zgyro = None
        self.sim_lat = None
        self.sim_lon = None
        self.sim_alt = None
        self.sim_vn = None
        self.sim_ve = None
        self.sim_vd = None
        self.sim_vx = None
        self.sim_vy = None
        self.sim_vz = None

    def set_data(self):
        pos, att, gps, vel, acc, ang, wind, sim = self.object.get_latest_data()

        if pos:
            self.pos_time, self.pos_x, self.pos_y, self.pos_z, self.pos_vx, self.pos_vy, self.pos_vz = zip(*pos)

        if att:
            self.att_time, self.roll, self.pitch, self.yaw, self.roll_speed, self.pitch_speed, self.yaw_speed = zip(*att)

        if gps:
            self.gps_time, self.gps_lat, self.gps_lon, self.gps_alt, self.gps_relative_alt, self.gps_vx, self.gps_vy, self.gps_vz, self.gps_hdg = zip(*gps)

        if acc:
            self.imu_time, self.imu_ax, self.imu_ay, self.imu_az = zip(*acc)

        if ang:
            self.imu_time, self.imu_wx, self.imu_wy, self.imu_wz = zip(*ang)

        if wind:
            self.wind_time, self.wind_speed, self.wind_direction = zip(*wind)

        if sim:
            self.sim_time, self.sim_roll, self.sim_pitch, self.sim_yaw, self.sim_xacc, self.sim_yacc, self.sim_zacc, self.sim_xgyro, self.sim_ygyro, self.sim_zgyro, self.sim_lat, self.sim_lon, self.sim_alt, self.sim_vn, self.sim_ve, self.sim_vd = zip(*sim)

