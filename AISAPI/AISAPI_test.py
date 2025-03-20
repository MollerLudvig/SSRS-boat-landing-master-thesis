import asyncio
import websockets
import json
from datetime import datetime, timezone

async def connect_ais_stream():

    async with websockets.connect("wss://stream.aisstream.io/v0/stream") as websocket:
        subscribe_message = {"APIKey": "1f2822724fc8865de6b262f69152cc31fdd2acc7",  # Required !
                             "BoundingBoxes": [[[-90, -180], [90, 180]]], # Required!
                             "FiltersShipMMSI": ["352004805"], # Vessel MMSI data (retrieved from AIS marinetraffic)
                             "FilterMessageTypes": ["PositionReport"]} # Optional!

        subscribe_message_json = json.dumps(subscribe_message)
        await websocket.send(subscribe_message_json)

        async for message_json in websocket:
            message = json.loads(message_json)
            message_type = message["MessageType"]

            if message_type == "PositionReport":
                # the message parameter contains a key of the message type which contains the message itself
                ais_message = message['Message']['PositionReport']
                print(f"Time: [{datetime.now(timezone.utc)}] \n" +
                      f"ShipId: {ais_message['UserID']} \n" +
                      f"Latitude: {ais_message['Latitude']} \n" +
                      f"Longitude: {ais_message['Longitude']} \n" +
                      f"Speed: {ais_message['Sog']} \n" +
                      f"Heading: {ais_message['TrueHeading']} \n")

if __name__ == "__main__":
    asyncio.run(connect_ais_stream())