import asyncio
import websockets
import json
import time
from datetime import datetime, timezone, timedelta
import pytz

class ShipTracker:
    def __init__(self,APIKey ="1f2822724fc8865de6b262f69152cc31fdd2acc7",  MMSI="265509950"):
        self.APIKey = APIKey
        self.MMSI = MMSI
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

        async with websockets.connect("wss://stream.aisstream.io/v0/stream") as websocket:
            subscribe_message = {"APIKey": self.APIKey,  # Required !
                                "BoundingBoxes": [[[-90, -180], [90, 180]]], # Required!
                                "FiltersShipMMSI": [self.MMSI], # Vessel MMSI data (retrieved from AIS marinetraffic)
                                "FilterMessageTypes": ["PositionReport"]} # Optional!

            subscribe_message_json = json.dumps(subscribe_message)
            await websocket.send(subscribe_message_json)

            async for message_json in websocket:
                message = json.loads(message_json)
                message_type = message["MessageType"]

                if message_type == "PositionReport":
                    # the message parameter contains a key of the message type which contains the message itself
                    ais_message = message['Message']['PositionReport']

                    self._manage_time(ais_message['Timestamp'])

                    ship.id = ais_message['UserID']
                    ship.lat = ais_message['Latitude']
                    ship.long = ais_message['Longitude']
                    ship.sog = ais_message['Sog'] * 0.514444 # Convert from knots to m/s
                    ship.yaw = ais_message['TrueHeading']

                    print(f'Time: {self.timestamp} \n' +
                        f'Time unix: {self.timestampUnix} \n' +
                        f'Time status: {self.timestatus} \n' +
                        f'Timestamp: {ais_message["Timestamp"]} \n' +
                        f'ShipId: {self.id} \n' +
                        f'Latitude: {self.lat} \n' +
                        f'Longitude: {self.long} \n' +
                        f'Speed: {self.sog} \n' +
                        f'Heading: {self.yaw} \n')

                    # print(f"Time: [{datetime.now(timezone.utc)}] \n" +
                    #     f"ShipId: {ais_message['UserID']} \n" +
                    #     f"Latitude: {ais_message['Latitude']} \n" +
                    #     f"Longitude: {ais_message['Longitude']} \n" +
                    #     f"Speed: {ais_message['Sog']} \n" +
                    #     f"Heading: {ais_message['TrueHeading']} \n")
                    

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
    ship = ShipTracker()
    asyncio.run(ship.connect_ais_stream())