import redis
import time
if __name__=="__main__":
    redis_cli = redis.Redis(host="localhost", port=6379)
    data="here1"
    data2="here2"
    redis_cli.set("channel",data)
    redis_cli.set("channel",data2)
