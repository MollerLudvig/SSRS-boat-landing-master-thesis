import asyncio
import json
import csv
import os
import time
from datetime import datetime, timezone, timedelta
import pytz
import pyais

class AISTracker:
    def __init__(self, MMSI="265547240", csvFileName="vipan"):
        self.MMSI = int(MMSI)
        self.csvFileName = csvFileName
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
            "ais.ssrs.se", 10023, "fredrik.falkman", "ssrs2027"
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

                    if mmsi == self.MMSI:
                        self.save_full_ais_data_csv(data)

    def save_full_ais_data_csv(self, data):
        directory = "data"
        os.makedirs(directory, exist_ok=True)

        if not self.sessionFileName:
            baseFilename = os.path.join(directory, self.csvFileName)
            fileNumber = 1
            while os.path.exists(f"{baseFilename}_{fileNumber}.csv"):
                fileNumber += 1
            self.sessionFileName = f"{baseFilename}_{fileNumber}.csv"

        # Add timestamp fields
        now = datetime.now(self.timezone)
        data["timestamp"] = now.isoformat()
        data["timestamp_unix"] = now.timestamp()

        if not self.fieldnames:
            self.fieldnames = list(data.keys())

        fileExists = os.path.exists(self.sessionFileName)

        with open(self.sessionFileName, "a", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)

            if not fileExists:
                writer.writeheader()
            writer.writerow(data)

    def _update_from_data(self, data):
        self.id = data.get("mmsi")
        self.lat = data.get("y")
        self.long = data.get("x")
        self.sog = data.get("sog") * 0.514444 if data.get("sog") is not None else None
        self.yaw = data.get("true_heading")
        self.rateOfTurn = data.get("rot")
        self.timestamp = datetime.now(self.timezone)
        self.timestampUnix = self.timestamp.timestamp()
        self.timestatus = "ok"

    def save_AIS_data_csv(self):
        directory = "data"
        os.makedirs(directory, exist_ok=True)

        if not hasattr(self, 'sessionFileName'):
            baseFilename = os.path.join(directory, self.csvFileName)
            fileNumber = 1
            while os.path.exists(f"{baseFilename}_{fileNumber}.csv"):
                fileNumber += 1
            self.sessionFileName = f"{baseFilename}_{fileNumber}.csv"

        fileExists = os.path.exists(self.sessionFileName)

        with open(self.sessionFileName, "a", newline="") as file:
            writer = csv.writer(file)
            if not fileExists:
                writer.writerow([
                    "Timestamp", "TimestampUnix", "Timestatus", "ShipId",
                    "Latitude", "Longitude", "Speed", "Heading", "RateOfTurn"
                ])
            writer.writerow([
                self.timestamp, self.timestampUnix, self.timestatus,
                self.id, self.lat, self.long, self.sog, self.yaw, self.rateOfTurn
            ])


if __name__ == "__main__":
    tracker = AISTracker(MMSI="265547240", csvFileName="vipan")

    async def main():
        await tracker.connect_ais_stream()

    asyncio.run(main())
