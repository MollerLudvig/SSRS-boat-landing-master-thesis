import asyncio
import websockets
import json
import time
import csv
import os
from datetime import datetime, timezone, timedelta
import pytz
from dotenv import load_dotenv

verbose = False



class AISTracker:
    def __init__(self, MMSI="265509950", csvFileName="AIS_data", apiKey=None):
        self.APIKey = apiKey if apiKey else os.getenv("AIS_API_KEY", "YOUR_DEFAULT_API_KEY")
        self.MMSI = MMSI
        self.csvFileName = csvFileName
        self.id = None
        self.lat = None
        self.long = None
        self.sog = None
        self.yaw = None
        self.rateOfTurn = None
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
                                        # "FilterMessageTypes": ["PositionReport"] # Optional men e la go enna
                    }

                    if verbose:
                        print(f"key{self.APIKey} connecting to AIS stream...")
                    
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

                            self.id = ais_message['UserID']
                            self.lat = ais_message['Latitude']
                            self.long = ais_message['Longitude']
                            self.sog = ais_message['Sog'] * 0.514444 # Convert from knots to m/s
                            self.yaw = ais_message['TrueHeading']
                            self.rateOfTurn = ais_message.get('RateOfTurn')

                            # Do something with the data
                            if verbose:
                                self.print_AIS_data()
                            else:
                                print(f"Ship {self.csvFileName} received PositionReport")
                            self.save_AIS_data_csv()


            except (websockets.exceptions.ConnectionClosedError, websockets.exceptions.ConnectionClosedOK) as e:
                print(f"Ship {self.csvFileName}. WebSocket connection lost: {e}. Reconnecting in 5 seconds...")
                await asyncio.sleep(5)
            except Exception as e:
                print(f"Ship {self.csvFileName}. Unexpected error: {e}. Reconnecting in 5 seconds...")
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
                writer.writerow([
                    "Timestamp", "TimestampUnix", "Timestatus", "ShipId",
                    "Latitude", "Longitude", "Speed", "Heading", "RateOfTurn"
                    ])
            writer.writerow([
                self.timestamp, self.timestampUnix, self.timestatus,
                self.id, self.lat, self.long, self.sog, self.yaw, self.rateOfTurn
            ])



    def print_AIS_data(self):
        """
        Print the AIS data in a readable format.
        """
        print("\nAIS data:")
        print(f"Time: {self.timestamp}")
        print(f"Time unix: {self.timestampUnix}")
        print(f"Time status: {self.timestatus}")
        print(f"ShipId: {self.id}")
        print(f"Latitude: {self.lat}")
        print(f"Longitude: {self.long}")
        print(f"Speed: {self.sog}")
        print(f"Heading: {self.yaw}")
        print(f"Rate of Turn: {self.rateOfTurn}")


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
            "rateOfTurn": self.rateOfTurn,
            "timestampUnix": self.timestampUnix,
            "timestamp": self.timestamp,
            "timestatus": self.timestatus
        }
                    

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
    
def load_api_keys(csv_path="api_keys.csv"):
    """Load API keys from a CSV file."""
    api_keys = []
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"API keys file '{csv_path}' not found.")
    
    with open(csv_path, newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            api_keys.append(row["api_key"].strip())  # Ensure no extra spaces
    
    if not api_keys:
        raise ValueError("No API keys found in CSV file.")

    return api_keys

if __name__ == "__main__":
    # List of MMSI numbers to track
    vessels = [
        # ("266460000", "fisk1"),
        # ("219340000", "fisk2"),
        # ("265650950", "rivo"),
        # ("265547210", "arlan"),
        # ("265509950", "vesta"),
        # ("265547270", "froja"),
        # ("265547230", "ylva"),
        # ("265547240", "vipan"),
        ("265650970", "valo")
        # ("265548670", "fjordska"),
        # ("265812010", "snuten"),
        # ("265547250", "skarven"),
        # ("265739650", "alvfrida"),
        # ("265029700", "elois"),
        # ("265501910", "hamnen")
    ]

    # Load API keys from CSV
    API_KEYS = load_api_keys()
    if verbose:
        print(f"Loaded {len(API_KEYS)} API keys from CSV.")

    # Assign API keys to each vessel in a round-robin fashion
    trackers = [
        AISTracker(MMSI=mmsi, csvFileName=name, apiKey=API_KEYS[i % len(API_KEYS)])
        for i, (mmsi, name) in enumerate(vessels)
    ]

    async def main():
        tasks = [asyncio.create_task(tracker.connect_ais_stream()) for tracker in trackers]
        await asyncio.gather(*tasks)

    asyncio.run(main())
