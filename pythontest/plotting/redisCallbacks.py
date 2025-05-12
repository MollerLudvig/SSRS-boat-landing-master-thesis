from redis_communication import RedisClient  # Import the modified RedisClient
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from threading import Thread, Lock

@dataclass
class PositionData:
    time: List[float] = field(default_factory=list)
    lat: List[float] = field(default_factory=list)
    lon: List[float] = field(default_factory=list)
    alt: List[float] = field(default_factory=list)

@dataclass
class DroneData:
    time: List[float] = field(default_factory=list)
    xs: List[float] = field(default_factory=list)
    altitude: List[float] = field(default_factory=list)
    z_wanted: List[float] = field(default_factory=list)
    needed_sr: List[float] = field(default_factory=list)
    wanted_sr: List[float] = field(default_factory=list)
    actual_sr: List[float] = field(default_factory=list)
    gr: List[float] = field(default_factory=list)

@dataclass
class BoatData:
    time: List[float] = field(default_factory=list)
    kf_x: List[float] = field(default_factory=list)
    kf_y: List[float] = field(default_factory=list)
    kf_lat: List[float] = field(default_factory=list)
    kf_lon: List[float] = field(default_factory=list)
    kf_speed: List[float] = field(default_factory=list)
    kf_heading: List[float] = field(default_factory=list)
    real_heading: List[float] = field(default_factory=list)
    real_lat: List[float] = field(default_factory=list)
    real_lon: List[float] = field(default_factory=list)

@dataclass
class FollowDiversionData:
    time: List[float] = field(default_factory=list)
    drone_distance: List[float] = field(default_factory=list)
    stall_speed: List[float] = field(default_factory=list)
    P2_distance: List[float] = field(default_factory=list)
    boat_speed: List[float] = field(default_factory=list)

@dataclass
class GrData:
    time: List[float] = field(default_factory=list)
    needed_gr: List[float] = field(default_factory=list)

@dataclass
class StageData:
    time: List[float] = field(default_factory=list)
    stage: List[str] = field(default_factory=list)


class RedisCallbacks:
    def __init__(self, redis_client: RedisClient):
        self.redis_client = redis_client
        self.P1 = PositionData()
        self.P2 = PositionData()
        self.P3 = PositionData()
        self.drone_data = DroneData()
        self.boat_data = BoatData()
        self.glide_ratio_data = GrData()
        self.StateData = StageData()
        self.follow_diversion_data = FollowDiversionData()
        
        # Setup callbacks for each position channel
        self.setup_callbacks()

    def setup_callbacks(self):
        """Register all callbacks with the Redis client"""
        self.redis_client.add_subscriber('P1', self.update_p1_position)
        self.redis_client.add_subscriber('P2', self.update_p2_position)
        self.redis_client.add_subscriber('P3', self.update_p3_position)
        self.redis_client.add_subscriber('drone data', self.update_drone_data)
        self.redis_client.add_subscriber('boat data', self.update_boat_data)
        self.redis_client.add_subscriber('follow diversion', self.update_follow_diversion_data)
        self.redis_client.add_subscriber('glide ratio', self.update_gr_data)
        self.redis_client.add_subscriber('stage', self.update_stage_data)
        print("Callbacks set up for Redis channels", flush=True)

    def update_p1_position(self, timestamp, content):
        """Callback for P1 position updates"""
        with Lock():
            if isinstance(content, dict) and all(k in content for k in ['time', 'lat', 'lon', 'alt']):
                self.P1.time.append(content['time'])
                self.P1.lat.append(content['lat'])
                self.P1.lon.append(content['lon'])
                self.P1.alt.append(content['alt'])
            else:
                print(f"Invalid data format for P1 position: {content}", flush=True)

    def update_p2_position(self, timestamp, content):
        """Callback for P2 position updates"""
        with Lock():
            if isinstance(content, dict) and all(k in content for k in ['time', 'lat', 'lon', 'alt']):
                
                self.P2.time.append(content['time'])
                self.P2.lat.append(content['lat'])
                self.P2.lon.append(content['lon'])
                self.P2.alt.append(content['alt'])
            else:
                print(f"Invalid data format for P2 position: {content}", flush=True)

    def update_p3_position(self, timestamp, content):
        """Callback for P3 position updates"""
        if isinstance(content, dict) and all(k in content for k in ['time', 'lat', 'lon', 'alt']):
            with Lock():
                self.P3.time.append(content['time'])
                self.P3.lat.append(content['lat'])
                self.P3.lon.append(content['lon'])
                self.P3.alt.append(content['alt'])
        else:
            print(f"Invalid data format for P3 position: {content}", flush=True)

    def update_drone_data(self, timestamp, content):
        """Callback for drone data updates"""
        with Lock():
            if isinstance(content, dict) and all(k in content for k in ['time', 'xs', 'altitude', 'z_wanted', 'needed_sr', 'wanted_sr', 'actual_sr', 'Gr']):
                self.drone_data.time.append(content['time'])
                self.drone_data.xs.append(content['xs'])
                self.drone_data.altitude.append(content['altitude'])
                self.drone_data.z_wanted.append(content['z_wanted'])
                self.drone_data.needed_sr.append(content['needed_sr'])
                self.drone_data.wanted_sr.append(content['wanted_sr'])
                self.drone_data.actual_sr.append(content['actual_sr'])
                self.drone_data.gr.append(content['Gr'])
            else:
                print(f"Invalid data format for drone data: {content}", flush=True)

    def update_boat_data(self, timestamp, content):
        """Callback for boat data updates"""
        with Lock():
            if isinstance(content, dict) and all(k in content for k in ['time', 'kf_x', 'kf_y', 'kf_lat', 'kf_lon', 'kf_speed', 'kf_heading', 'real_heading', 'real_lat', 'real_lon']):
                self.boat_data.time.append(content['time'])
                self.boat_data.kf_x.append(content['kf_x'])
                self.boat_data.kf_y.append(content['kf_y'])
                self.boat_data.kf_lat.append(content['kf_lat'])
                self.boat_data.kf_lon.append(content['kf_lon'])
                self.boat_data.kf_speed.append(content['kf_speed'])
                self.boat_data.kf_heading.append(content['kf_heading'])
                self.boat_data.real_heading.append(content['real_heading'])
                self.boat_data.real_lat.append(content['real_lat'])
                self.boat_data.real_lon.append(content['real_lon'])
            else:
                print(f"Invalid data format for boat data: {content}", flush=True)

    def update_follow_diversion_data(self, timestamp, content):
        """Callback for follow diversion data updates"""
        with Lock():
            if isinstance(content, dict) and all(k in content for k in ['time', 'drone_distance', 'stall_speed', 'P2_distance', 'boat_speed']):
                self.follow_diversion_data.time.append(content['time'])
                self.follow_diversion_data.drone_distance.append(content['drone_distance'])
                self.follow_diversion_data.stall_speed.append(content['stall_speed'])
                self.follow_diversion_data.P2_distance.append(content['P2_distance'])
                self.follow_diversion_data.boat_speed.append(content['boat_speed'])
            else:
                print(f"Invalid data format for follow diversion data: {content}", flush=True)
    
    def update_gr_data(self, timestamp, content):
        """Callback for glide ratio data updates"""
        with Lock():
            if isinstance(content, dict) and all(k in content for k in ['time', 'needed_gr']):
                self.glide_ratio_data.time.append(content['time'])
                self.glide_ratio_data.needed_gr.append(content['needed_gr'])
            else:
                print(f"Invalid data format for glide ratio data: {content}", flush=True)

    def update_stage_data(self, timestamp, content):
        """Callback for stage data updates"""
        with Lock():
            if isinstance(content, dict) and all(k in content for k in ['time', 'stage']):
                self.StateData.time.append(content['time'])
                self.StateData.stage.append(content['stage'])
            else:
                print(f"Invalid data format for stage data: {content}", flush=True)

    def start_listening(self):
        """Start listening for Redis messages in a separate thread"""
        print("Starting Redis listener in background thread", flush=True)
        self.listener_thread = Thread(target=self.redis_client.listen, daemon=True)
        self.listener_thread.start()
        print("Redis listener started in background thread", flush=True)
        


# Example usage:
def main():
    # Initialize the Redis client
    redis_client = RedisClient(host="localhost", port=6379)
    
    # Initialize callbacks
    callbacks = RedisCallbacks(redis_client)
    
    # Start listening in a non-blocking way
    callbacks.start_listening()
    
    print("Main thread is free to do other operations")
    
    # Example of sending test messages
    import time
    time.sleep(1)  # Give the listener time to start
    
    # Send a test message
    redis_client.send_message('P1_position', {
        'time': time.time(),
        'lat': 37.7749,
        'lon': -122.4194,
        'alt': 100.0
    })
    
    # Continue with other operations in the main thread
    time.sleep(5)
    
    # Print the collected data
    if callbacks.P1.lat:
        print(f"Latest P1 latitude: {callbacks.P1.lat[-1]}")
    else:
        print("No P1 position data received yet")
    
    # Clean shutdown
    callbacks.stop_listening()

if __name__ == "__main__":
    main()