import redis
import time

redis_cli = redis.Redis(host="localhost", port=16379)

p = redis_cli.pubsub()

p.subscribe("my-channel-1", "my-channel-2")

while True:
    message = p.get_message()
    if message:
        print("here")
        print(message)
    time.sleep(0.01)