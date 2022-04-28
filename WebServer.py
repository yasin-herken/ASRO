import redis
import json
from flask import Flask, request, Response
from flask_cors import CORS, cross_origin
from yaml import parse


app = Flask(__name__)
app.config['CORS_HEADERS'] = 'Content-Type'
cors = CORS(app)
redisClient = redis.Redis()

@app.route('/')
@cross_origin()
def home():
    return Response("Flask server is up an running!\n", 200)

@app.route('/mission_one')
@cross_origin()
def mission_one():
    redisClient.publish("requests", json.dumps({"mission": "mission_one"}))
    
    return Response("Done!", 200)

@app.route('/mission_two')
@cross_origin()
def mission_two():
    redisClient.publish("requests", json.dumps({"mission": "mission_two"}))
        
    return Response("Done!", 200)
    
@app.route('/mission_three')
@cross_origin()
def mission_three():
    redisClient.publish("requests", json.dumps({"mission": "mission_three"}))
        
    return Response("Done!", 200)

@app.route('/mission_four')
@cross_origin()
def mission_four():
    redisClient.publish("requests", json.dumps({"mission": "mission_four"}))
        
    return Response("Done!", 200)

@app.route('/mission_five')
@cross_origin()
def mission_five():
    redisClient.publish("requests", json.dumps({"mission": "mission_five"}))

@app.route('/mission_takeoff')
@cross_origin()
def mission_takeoff():
    target = request.args.get('target')
    
    redisClient.publish("requests", json.dumps({"mission": "mission_takeoff", "target": target}))
        
    return Response(f"{target}\n", 200)

@app.route('/mission_land')
@cross_origin()
def mission_land():
    target = request.args.get('target')
    
    redisClient.publish("requests", json.dumps({"mission": "mission_land", "target": target}))
        
    return Response(f"{target}\n", 200)

@app.route('/mission_takeoff_all')
@cross_origin()
def mission_take_off_all():
    
    redisClient.publish("requests", json.dumps({"mission": "mission_takeoff_all"}))
        
    return Response(f"Done\n", 200)

@app.route('/mission_land_all')
@cross_origin()
def mission_land_all():
    
    redisClient.publish("requests", json.dumps({"mission": "mission_land_all"}))
        
    return Response(f"Done\n", 200)

@app.route('/get_pose')
@cross_origin()
def get_pose():    
    result = {
        "posX": 34.64,
        "posY": 0.3,
        "posZ": 1.5,
        "state": "MOVING"
    }
        
    return Response(
        response=json.dumps(result),
        status=200,
        mimetype='application/json'
    )
    
@app.route('/get_agents')
@cross_origin()
def get_agents():    
    rawData = redisClient.get("agents")
    names = []
    addreses = []
    
    if rawData:
        parsed = json.loads(rawData)
        names = parsed.get("names")
        addreses = parsed.get("addresses")
    
    result = {
        "names": names,
        "addresses": addreses
    }
    
    return Response(
        response=json.dumps(result),
        status=200,
        mimetype='application/json'
    )

if __name__ == "__main__":
    app.run(host='0.0.0.0', debug=True)