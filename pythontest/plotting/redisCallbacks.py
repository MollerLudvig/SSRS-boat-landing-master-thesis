import threading
from redis_communication import RedisClient
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

@dataclass
class PositionData:
    time: List[float] = field(default_factory=list)
    lat: List[float] = field(default_factory=list)
    lon: List[float] = field(default_factory=list)
    alt: List[float] = field(default_factory=list)


class RedisCallbacks:
    def __init__(self, redis_client: RedisClient):
        self.redis_client = redis_client
        self.P1_data = PositionData()
        self.P2_data = PositionData()
        self.P3_data = PositionData()
        
        # Setup callbacks for each position channel
        self.setup_callbacks()
        
        # Create a separate thread for listening to Redis messages
        self.listener_thread = None

    def setup_callbacks(self):
        self.redis_client.add_subscriber('P1', self.update_P1)
        self.redis_client.add_subscriber('P2', self.update_P2)
        self.redis_client.add_subscriber('P3', self.update_P3)


    def update_P1(self, timestamp, content):
        """Callback for P1 updates"""
        if isinstance(content, dict) and all(k in content for k in ['time', 'lat', 'lon', 'alt']):
            self.P1_data.time.append(content['time'])
            self.P1_data.lat.append(content['lat'])
            self.P1_data.lon.append(content['lon'])
            self.P1_data.alt.append(content['alt'])
        else:
            print(f"Invalid data format for P1 position: {content}", flush=True)

    def update_P2(self, timestamp, content):
        """Callback for P2 updates"""
        if isinstance(content, dict) and all(k in content for k in ['time', 'lat', 'lon', 'alt']):
            self.P2_data.time.append(content['time'])
            self.P2_data.lat.append(content['lat'])
            self.P2_data.lon.append(content['lon'])
            self.P2_data.alt.append(content['alt'])
        else:
            print(f"Invalid data format for P2 position: {content}", flush=True)

    def update_P3(self, timestamp, content):
        """Callback for P3 updates"""
        if isinstance(content, dict) and all(k in content for k in ['time', 'lat', 'lon', 'alt']):
            self.P3_data.time.append(content['time'])
            self.P3_data.lat.append(content['lat'])
            self.P3_data.lon.append(content['lon'])
            self.P3_data.alt.append(content['alt'])
        else:
            print(f"Invalid data format for P3 position: {content}", flush=True)

    def start_listening(self):
        """Start listening for Redis messages in a separate thread"""
        if self.listener_thread is None or not self.listener_thread.is_alive():
            self.listener_thread = threading.Thread(target=self.redis_client.listen)
            self.listener_thread.daemon = True  # Thread will terminate when the main program exits
            self.listener_thread.start()
            print("Redis listener started in background thread", flush=True)
        else:
            print("Redis listener is already running", flush=True)


    