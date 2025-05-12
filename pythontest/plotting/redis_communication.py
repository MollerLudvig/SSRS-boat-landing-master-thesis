import redis
import json
import ast
import time


class RedisClient:
    """
    Initializes the RedisClient.
    """

    def __init__(
            self,
            host: str = "localhost",
            port: int = 6379,
    ):
        self._r = redis.Redis(host=host, port=port, db=0)
        self._callbacks = {}
        self._last_stream_ids = {}

        # Make 3 attempts to connect to redis server, raise error if all fails
        for attempt in range(3):
            try:
                self._r.ping()
            except redis.exceptions.ConnectionError as e:
                if attempt == 2:
                    print(f"Redis {e}", flush=True)
                    exit(1)
                print("Redis server is not responding. Sleeping...", flush=True)
                time.sleep(5)

                continue
            print(f"Connected to Redis server at {host}:{port}", flush=True)
            break

        self._pubsub = self._r.pubsub()

    def add_subscriber(
            self,
            channel_name: str,
            callback: callable
    ):
        """
        Adds a subscriber to a specified Redis channel and sets a callback function.

        :param channel_name: The name of the Redis channel to subscribe to.
        :param callback: A callable function to handle messages received from the channel. Should accept two arguments: timestamp and content.
        """

        # return if channel has already been added
        if channel_name in self._callbacks:
            print(f'Error: channel "{channel_name}" already exists.', flush=True)
            return
        # save subscription
        self._callbacks[channel_name] = callback
        self._pubsub.subscribe(channel_name)

    def remove_subscriber(
            self,
            channel_name: str
    ):
        """
        Removes a subscriber from a specified Redis channel.

        :param channel_name: The name of the Redis channel to unsubscribe from.
        """

        try:
            self._callbacks.pop(channel_name)
        except KeyError:
            return
        self._pubsub.unsubscribe(channel_name)

    def send_message(
            self,
            channel_name: str,
            content: str | int | float | dict | list
    ):
        """
        Sends a message to a Redis channel.

        :param channel_name: The name of the Redis channel to publish the message.
        :param content: The content to be sent (must be serializable to JSON).
        """

        # add timestamp
        content = {"timestamp": self._r.time(), "content": content}
        content = self._to_json(content)
        if content is not None:
            # send message
            self._r.publish(channel_name, content)

    def listen(self):
        """
         Starts listening for incoming messages on subscribed channels and executes their respective callbacks.

         Note: This method runs an infinite loop to process incoming messages.
         """

        for event in self._pubsub.listen():
            if event["type"] == "message":
                channel_name = event["channel"].decode("utf-8")
                message = self._from_json(event["data"])
                if not {'timestamp', 'content'} == message.keys():
                    print(f"Error: message should contain 'timestamp' and 'content' but got: {message}", flush=True)
                    continue
                try:
                    self._callbacks[channel_name](message['timestamp'], message['content'])
                except TypeError:
                    print(f"Error: callback for channel '{channel_name}' has invalid signature. Should be (timestamp:float, content: Any)", flush=True)
                    continue

    def add_stream_message(
            self,
            stream_name: str,
            content: str | int | float | dict | list
    ):
        """
        Adds a message to a specified Redis stream.

        :param stream_name: The name of the Redis stream.
        :param content: The content to add to the stream (must be serializable to JSON).
        """

        content = self._to_json(content)
        self._r.xadd(stream_name, {'timestamp': str(self._r.time()), 'content': content})

    def get_latest_stream_message(
            self,
            stream_name: str
    ) -> tuple[tuple[int, int], str | int | float | dict | list] | tuple[None, None]:
        """
        Retrieves the latest message from a Redis stream.

        :param stream_name: The name of the Redis stream.
        :return: A tuple containing the timestamp and the message content, or (None, None) if no messages exist.
        """

        # read the latest message
        raw_message = self._r.xrevrange(stream_name, count=1)
        # return None if no messages
        if len(raw_message) == 0:
            return None, None
        # latest message
        raw_message = raw_message[0]
        # save last read stream id
        self._last_stream_ids[stream_name] = raw_message[0].decode("utf-8")

        return self._decode_stream_message(raw_message)

    def get_unread_stream_messages(
            self,
            stream_name: str,
            max_messages: int = None
    ) -> list[tuple[tuple[int, int], str | int | float | dict | list]] | list[tuple[None, None]]:
        """
        Retrieves unread messages from a Redis stream, starting from the last-read stream ID.

        :param stream_name: The name of the Redis stream.
        :param max_messages: The maximum number of unread messages to fetch.
        :return: List of tuples (timestamp, content) or (None, None) if no unread messages exist.
        """

        messages = []
        # if it is the first time, get all messages (indicated by 0)
        stream_id = self._last_stream_ids.get(stream_name, "0")
        raw_messages = self._r.xrange(stream_name, stream_id, "+", count=max_messages)
        if len(raw_messages) == 0:
            return [(None, None)]
        # if it not the first time, omit the first message in list because xrange is inclusive
        if stream_id != "0":
            raw_messages = raw_messages[1:]
        if len(raw_messages) == 0:
            return [(None, None)]

        # save the newest stream id
        self._last_stream_ids[stream_name] = raw_messages[-1][0].decode("utf-8")
        for r in raw_messages:
            messages.append(self._decode_stream_message(r))
        return messages

    @staticmethod
    def _to_json(
            message: str | int | float | dict | list
    ) -> str | None:
        """
        Converts a Python object into a JSON-formatted string.

        :param message: The Python object to be serialized (str, int, float, dict, list, etc.).
        :return: JSON-formatted string or None if serialization fails.
        """

        # test if json serializable
        try:
            return json.dumps(message)
        except TypeError:
            print(f"Error: message is not JSON serializable: {message}", flush=True)
            return None

    @staticmethod
    def _from_json(
            message: str
    ) -> str | int | float | dict | list:
        """
        Decodes a JSON-formatted string into a Python object.

        :param message: JSON-formatted string to decode.
        :return: Decoded Python object (dict, list, etc.) or the input unchanged if decoding fails.
        """

        try:
            return json.loads(message)
        except json.decoder.JSONDecodeError as e:
            print(f"Error: could not read message: {message}. {e}", flush=True)
        except TypeError as e:
            print(e, flush=True)
        return message

    @staticmethod
    def _decode_stream_message(
            message: list[dict[bytes]]
    ) -> tuple[tuple[int, int], str | int | float | dict | list]:
        """
        Decodes raw Redis stream messages into tuples with timestamps and content.

        :param raw_messages: List of raw stream messages from Redis, each containing an ID and fields (dictionary).
        :return: List of tuples (timestamp, content).
        """

        # decode timestamp and message
        timestamp = ast.literal_eval(message[1][b"timestamp"].decode("utf-8"))
        content = RedisClient._from_json(message[1][b"content"])
        return timestamp, content
