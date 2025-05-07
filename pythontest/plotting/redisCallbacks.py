from redis_communication import RedisClient  # Import the modified RedisClient
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
        self.P1 = PositionData()
        self.P2 = PositionData()
        self.P3 = PositionData()
        
        # Setup callbacks for each position channel
        self.setup_callbacks()

    def setup_callbacks(self):
        """Register all callbacks with the Redis client"""
        self.redis_client.add_subscriber('P1_position', self.update_p1_position)
        self.redis_client.add_subscriber('P2_position', self.update_p2_position)
        self.redis_client.add_subscriber('P3_position', self.update_p3_position)

    def update_p1_position(self, timestamp, content):
        """Callback for P1 position updates"""
        if isinstance(content, dict) and all(k in content for k in ['time', 'lat', 'lon', 'alt']):
            self.P1.time.append(content['time'])
            self.P1.lat.append(content['lat'])
            self.P1.lon.append(content['lon'])
            self.P1.alt.append(content['alt'])
        else:
            print(f"Invalid data format for P1 position: {content}", flush=True)

    def update_p2_position(self, timestamp, content):
        """Callback for P2 position updates"""
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
            self.P3.time.append(content['time'])
            self.P3.lat.append(content['lat'])
            self.P3.lon.append(content['lon'])
            self.P3.alt.append(content['alt'])
        else:
            print(f"Invalid data format for P3 position: {content}", flush=True)

    def start_listening(self):
        """Start listening for Redis messages in a non-blocking way"""
        if self.redis_client.start_listening_thread():
            print("Redis listener started in background thread", flush=True)
        else:
            print("Redis listener is already running", flush=True)

    def stop_listening(self):
        """Stop the Redis listener thread"""
        self.redis_client.stop_listening()
        print("Redis listener stopped", flush=True)


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