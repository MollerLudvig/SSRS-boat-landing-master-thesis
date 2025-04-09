import asyncio
import websockets
import json
import time
import csv
import os
from datetime import datetime, timezone, timedelta
import pytz
import pyais
from dotenv import load_dotenv

verbose = True



class AISTracker:
    def __init__(self, MMSI="265547240", csvFileName="vipan"):
        self.MMSI = MMSI
        self.csvFileName = csvFileName
        self.data = {}
        self.timestampUnix = time.time()
        self.timestamp = datetime.now(timezone.utc)
        self.timestatus = "Uninitiated"
        self.timezone = pytz.timezone("Europe/Stockholm")
        self.sessionFileName = None
        self.headerWritten = False
        self.fieldnames = []

    async def tcp_login(self, addr, port, username, password):
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(addr, port), timeout=5
            )
        except asyncio.TimeoutError:
            print(f"Timeout connecting to AIS server at {addr}:{port}")
            return None, None

        login_msg = (
            bytes([1])
            + username.encode("utf-8")
            + bytes([0])
            + password.encode("utf-8")
            + bytes([0])
        )

        writer.write(login_msg)
        await writer.drain()
        return reader, writer
    
    async def connect_ais_stream(self):
        reader, _ = await self.tcp_login(
            "ais.ssrs.se", 10024, "fredrik.falkman", "ssrs2027"
        )

        if reader is None:
            return

        buf = b""
        while True:
            msg = await reader.readline()
            if msg == b"":
                print("AIS server closed connection")
                break

            if msg.startswith(b"!ABVDM"):
                try:
                    if buf:
                        ais_msg = pyais.decode(buf, msg)
                        buf = b""
                    else:
                        ais_msg = pyais.decode(msg)
                except pyais.exceptions.MissingMultipartMessageException:
                    buf += msg
                    continue

                # Only save message if complete
                if buf == b"":
                    data = json.loads(ais_msg.to_json())
                    mmsi = data.get("mmsi")
                    
                    # Save message if MMSI matches
                    if str(mmsi) == str(self.MMSI):
                        # Print the message if verbose is enabled
                        if verbose:
                            print(f"Received AIS message: {data}")

                        self.data = data
                        
                        # Add speed in m/s
                        self.data["speed[m/s]"] = data.get("speed") * 0.514444

                        # Add timestamps
                        self._manage_time()

                        self.save_full_ais_data_csv()

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

    def save_full_ais_data_csv(self):
        directory = "data"
        os.makedirs(directory, exist_ok=True)

        if not self.sessionFileName:
            baseFilename = os.path.join(directory, self.csvFileName)
            fileNumber = 1
            while os.path.exists(f"{baseFilename}_{fileNumber}.csv"):
                fileNumber += 1
            self.sessionFileName = f"{baseFilename}_{fileNumber}.csv"

        # Update the data dictionary with the new values
        if not self.fieldnames:
            self.fieldnames = list(self.data.keys())

        fileExists = os.path.exists(self.sessionFileName)

        with open(self.sessionFileName, "a", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)

            if not fileExists:
                writer.writeheader()
            writer.writerow(self.data)


    def _manage_time(self):
        currentTime = datetime.now(self.timezone)
        second = self.data.get("second")

        # Check if second is int and in range
        if isinstance(second, int) and 0 <= second <= 59:
            self.timestamp = currentTime.replace(second=second, microsecond=0)
            
            # Timestamp ahead of current time indicates rollover on minute between measurement and an received message
            if currentTime < self.timestamp:
                self.timestamp -= timedelta(minutes=1)

            self.timestampUnix = self.timestamp.timestamp()
            self.timestatus = "ok"
        else:
            # Second outside range indicates other timestamp status
            self.timestamp = None
            self.timestampUnix = None
            match second:
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

        self.data["timestamp"] = self.timestamp
        self.data["timestamp_unix"] = self.timestampUnix
        self.data["timestatus"] = self.timestatus


if __name__ == "__main__":
    # List of MMSI numbers to track

    tracker = AISTracker(MMSI="265547230", csvFileName="ylva")

    async def main():
        await tracker.connect_ais_stream()

    asyncio.run(main())