import asyncio
import websockets
import json
import time
import csv
import os
from datetime import datetime, timezone, timedelta
import pytz
from dotenv import load_dotenv

# Load api key from env/.env file
env_path = os.path.join("env", ".env")
load_dotenv(env_path)

class AISTracker:
    def __init__(self, MMSI="265509950", csvFileName="AIS_data"):
        self.APIKey = os.getenv("AIS_API_KEY", "YOUR_DEFAULT_API_KEY") 
        self.MMSI = MMSI
        self.csvFileName = csvFileName
        self.id = None
        self.lat = None
        self.long = None
        self.sog = None
        self.yaw = None
        self.timestampUnix = time.time()
        self.timestamp = datetime.now(timezone.utc)
        self.timestatus = "Uninitiated"
        self.timezone = pytz.timezone("Europe/Stockholm")


    async def connect_ais_stream(self):

        while True:
            try:
                async with websockets.connect("wss://stream.aisstream.io/v0/stream") as websocket:
                    subscribe_message = {"APIKey": self.APIKey,  # Required
                                        "BoundingBoxes": [[[-90, -180], [90, 180]]], # Required
                                        "FiltersShipMMSI": [self.MMSI], # Vessel MMSI data (retrieved from AIS marinetraffic)
                                        "FilterMessageTypes": ["PositionReport"]} # Optional men e la go enna

                    subscribe_message_json = json.dumps(subscribe_message)
                    await websocket.send(subscribe_message_json)

                    async for message_json in websocket:
                        message = json.loads(message_json)
                        message_type = message["MessageType"]

                        if message_type == "PositionReport":
                            # the message parameter contains a key of the message type which contains the message itself
                            ais_message = message['Message']['PositionReport']


                            # Check if the message is for the correct MMSI
                            self._manage_time(ais_message['Timestamp'])

                            AIS.id = ais_message['UserID']
                            AIS.lat = ais_message['Latitude']
                            AIS.long = ais_message['Longitude']
                            AIS.sog = ais_message['Sog'] * 0.514444 # Convert from knots to m/s
                            AIS.yaw = ais_message['TrueHeading']

                            # Do something with the data
                            self.print_AIS_data()
                            self.save_AIS_data_csv()


            except (websockets.exceptions.ConnectionClosedError, websockets.exceptions.ConnectionClosedOK) as e:
                print(f"WebSocket connection lost: {e}. Reconnecting in 5 seconds...")
                await asyncio.sleep(5)
            except Exception as e:
                print(f"Unexpected error: {e}. Reconnecting in 5 seconds...")
                await asyncio.sleep(5)

    def save_AIS_data_csv(self):
        directory = "data"
        os.makedirs(directory, exist_ok=True)

        # Generate the session file name only once per run
        if not hasattr(self, 'sessionFileName'):
            baseFilename = os.path.join(directory, self.csvFileName)
            fileNumber = 1
            while os.path.exists(f"{baseFilename}_{fileNumber}.csv"):
                fileNumber += 1
            self.sessionFileName = f"{baseFilename}_{fileNumber}.csv"

        fileExists = os.path.exists(self.sessionFileName)

        with open(self.sessionFileName, "a", newline="") as file:
            writer = csv.writer(file)
            if not fileExists:  # Write header only if file didn't exist previously
                writer.writerow(["Timestamp", "TimestampUnix", "Timestatus", "ShipId", "Latitude", "Longitude", "Speed", "Heading"])
            writer.writerow([
                self.timestamp, self.timestampUnix, self.timestatus,
                self.id, self.lat, self.long, self.sog, self.yaw
            ])



    def print_AIS_data(self):
        """
        Print the AIS data in a readable format.
        """
        print("\nAIS data:")
        print(f'Time: {self.timestamp}\n' +
              f'Time unix: {self.timestampUnix}\n' +
              f'Time status: {self.timestatus}\n' +
              f'ShipId: {self.id}\n' +
              f'Latitude: {self.lat}\n' +
              f'Longitude: {self.long}\n' +
              f'Speed: {self.sog}\n' +
              f'Heading: {self.yaw}\n')

    def print_AIS_data_dict(self):
        """
        Print the AIS data as a dictionary.
        """
        AIS_data = self.get_AIS_data()
        print(AIS_data)



    """
    Getters for AIS data
    """

    def get_AIS_data(self):
        """
        Returns the AIS data as a dictionary.
        """
        return {
            "id": self.id,
            "lat": self.lat,
            "long": self.long,
            "sog": self.sog,
            "yaw": self.yaw,
            "timestampUnix": self.timestampUnix,
            "timestamp": self.timestamp,
            "timestatus": self.timestatus
        }
    
    def get_AIS_id(self):
        return self.id
    
    def get_AIS_lat(self):
        return self.lat
    
    def get_AIS_long(self):
        return self.long
    
    def get_AIS_sog(self):
        return self.sog
    
    def get_AIS_yaw(self):
        return self.yaw
    
    def get_AIS_timestamp(self):
        return self.timestamp
    
    def get_AIS_timestamp_unix(self):
        return self.timestampUnix
    
    def get_AIS_timestatus(self):
        return self.timestatus
    
    def get_AIS_timezone(self):
        return self.timezone
    
    def get_AIS_APIKey(self):
        return self.APIKey
    
    def get_AIS_MMSI(self):
        return self.MMSI

                    

    def _manage_time(self, timestamp):
        """
        Convert timestamp to datetime object and Unix timestamp.
        """
        currentTime = datetime.now(self.timezone)

        if 0 <= timestamp <= 59:
            # Replace swedish time with correct second 
            self.timestamp = currentTime.replace(second=timestamp, microsecond=0)
            # If the timestamp is in the future, subtract a minute
            if currentTime < self.timestamp:
                self.timestamp -= timedelta(minutes=1)

            # Convert to Unix timestamp
            self.timestampUnix = self.timestamp.timestamp()
            self.timestatus = "ok"

        else:
            # Handle special cases for AIS timestamps
            self.timestamp = None
            self.timestampUnix = None
            match timestamp:
                case 60:
                    self.timestatus = "Time stamp is not available"
                case 61:
                    self.timestatus = "Positioning system is in manual input mode"
                case 62:
                    self.timestatus = "Positioning system is in estimated (dead reckoning) mode"
                case 63:
                    self.timestatus = "Positioning is inoperative"
                case _:
                    self.timestatus = "Unknown timestamp value"

if __name__ == "__main__":
    AIS = AISTracker()
    asyncio.run(AIS.connect_ais_stream())