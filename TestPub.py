import redis
import json
import marshal
redis_cli = redis.Redis(host='localhost', port=6379)
product_stock={
    '__name':'cf1',
    '__adress':'crazyradio',
    '__status':'online',
    '__state':'HOVERING',
    '__pos':[0,0,0],
    '__vel':[1,2,3],
    '__speed':1.2,
    '__pitch':1.3,
    '__yaw':1.4,
    '__row':1.5
}
data=marshal.dumps(product_stock)
redis_cli.set("product_stock",data)

data=redis_cli.get("product_stock")
new_data=marshal.loads(data)
print(new_data['__name'])
print(type(new_data))
