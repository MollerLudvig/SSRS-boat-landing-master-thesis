import threading
from time import sleep
from redis_communication import RedisClient

rc = RedisClient()
stage_stream = "stage"

def input_thread():
    while True:
        user_input = input("follow or land: ")
        rc.add_stream_message(stage_stream, user_input)

thread = threading.Thread(target=input_thread, daemon=True)
thread.start()

while True:
    wanted_stage = rc.get_latest_stream_message("stage")[1]
    if wanted_stage == "land":
        Gr = rc.get_latest_stream_message("needed_glide_ratio")[1]
        if Gr > 0.25: 
            rc.add_stream_message(stage_stream, "diversion") 

    if wanted_stage == "follow":
        # Read P1 dist, drone dist, boat speed and figure out if diversion is needed
        follow_diversion_data = rc.get_latest_stream_message("follow diversion")[1]

        drone_distance_to_boat = follow_diversion_data["drone_distance"]
        stall_speed = follow_diversion_data["stall_speed"]
        P2_distance = follow_diversion_data["P2_distance"]
        boat_speed = follow_diversion_data["boat_speed"]

        if boat_speed < stall_speed and drone_distance_to_boat < (P2_distance + 5):
            rc.add_stream_message(stage_stream, "diversion")
    
    if wanted_stage == "FBWB":
        rc.add_stream_message(stage_stream, "FBWB")

