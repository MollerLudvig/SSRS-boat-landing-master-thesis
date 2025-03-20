from time import sleep
from redis_communication import RedisClient

rc = RedisClient()
stream_name = "stage"

while True:
    user_input = input("follow or land: ")
    rc.add_stream_message(stream_name, user_input)


