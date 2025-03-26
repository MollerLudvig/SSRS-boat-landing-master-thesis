import asyncio
import websockets
import json
from datetime import datetime, timezone
from datetime import datetime
import pytz
swedish_tz = pytz.timezone("Europe/Stockholm")
swedish_time = datetime.now(swedish_tz)

async def connect_ais_stream():

    async with websockets.connect("wss://stream.aisstream.io/v0/stream") as websocket:
        subscribe_message = {"APIKey": "1f2822724fc8865de6b262f69152cc31fdd2acc7",  # Required !
                             "BoundingBoxes": [[[-90, -180], [90, 180]]], # Required!
                             "FiltersShipMMSI": ["265739650"], # Vessel MMSI data (retrieved from AIS marinetraffic)
                             "FilterMessageTypes": ["PositionReport"]} # Optional!

        subscribe_message_json = json.dumps(subscribe_message)
        await websocket.send(subscribe_message_json)

        async for message_json in websocket:
            message = json.loads(message_json)
            message_type = message["MessageType"]

            if message_type == "PositionReport":
                ais_message = message['Message']['PositionReport']
                timestamp = ais_message['Timestamp']
                current_time = datetime.now(timezone.utc)

                if 0 <= timestamp <= 59:
                    # Construct the correct timestamp
                    corrected_time = swedish_time.replace(second=timestamp, microsecond=0)
                else:
                    # Handle special cases for AIS timestamps
                    match timestamp:
                        case 60:
                            timestamp_info = "Time stamp is not available"
                        case 61:
                            timestamp_info = "Positioning system is in manual input mode"
                        case 62:
                            timestamp_info = "Positioning system is in estimated (dead reckoning) mode"
                        case 63:
                            timestamp_info = "Positioning is inoperative"
                        case _:
                            timestamp_info = "Unknown timestamp value"

                    corrected_time = f"[{timestamp_info}]"

                print(f"Time Received: [{current_time}] \n" +
                      f"AIS Timestamp: {timestamp} \n" +
                      f"Corrected Time: {corrected_time} \n" +
                      f"ShipId: {ais_message['UserID']} \n" +
                      f"Latitude: {ais_message['Latitude']} \n" +
                      f"Longitude: {ais_message['Longitude']} \n" +
                      f"Speed: {ais_message['Sog']} kn\n" +
                      f"Heading: {ais_message['TrueHeading']}° \n")

if __name__ == "__main__":
    asyncio.run(connect_ais_stream())