from ctypes import addressof
import os
import sys
import redis
import time
import Settings
import json
import numpy as np

from Agent import Agent
from MissionControl import MissionControl
from pycrazyswarm import Crazyswarm


def _watchDog(server: Crazyswarm) -> None:
    """Runs on a thread. Checks the Redis 'emergency' channel.
    Calls the function emergencyExit() if the need arises.

    Args:
        server (Crazyswarm): To be used when calling emergencyExit().
    """
    pass

def emergencyExit(server: Crazyswarm) -> None:
    """Kills the Crazyswarm server and turns off all the agents.

    Args:
        server (Crazyswarm): Server to be killed.
    """
    pass

def main() -> None:
    """Entry point for the ASRO software. Initializes the Crazyswarm server.
    Creates Agents and the Redis server which will be used by the WebApplication.
    
    Checks the Redis 'control' channel for any new mission requests.
    If a new request exists, hands over the control to MissionControl.
    """
    # Initialization
    cfIds = []
    agents = []
    names = []
    addresses = []
    statuses = []
    
    crazySwarm = Crazyswarm(crazyflies_yaml="./crazyflies.yaml")
    
    # Geting the agent ids
    for id in crazySwarm.allcfs.crazyfliesById:
        cfIds.append(id)
    
    # Creating the agents
    i = 0
    for agent in crazySwarm.allcfs.crazyflies:
        agents.append(
            Agent(
                cf=agent,
                name=f"Agent {cfIds[i]}",
                address=f"radio:/{cfIds[i]}/110/2M"
            )
        )
        names.append(f"Agent {cfIds[i]}")
        addresses.append(f"radio:/{cfIds[i]}/110/2M")
        i+=1
    
    # Initializing redis
    redisClient = redis.Redis()
    redisClient.set("agents", json.dumps({"names": names, "addresses": addresses}))
    redisSub = redisClient.pubsub()
    redisSub.subscribe(["requests"])
    
    # Creating mission control    
    missionControl = MissionControl(
        crazySwarm=crazySwarm,
        agents=agents,
        redisClient=redisClient
    )
    
    while True:
        # Read incoming messages from redis
        msg = redisSub.get_message()
        mission = ""
        target = ""
        
        # Parse the message
        if msg:
            if msg['type'] == 'message':
                mission = json.loads(msg['data']).get("mission", None)
                target = json.loads(msg['data']).get("target", None)
        
        # Launch a mission if a message exists
        if mission == "mission_takeoff_all":
            print("hereee")
            missionControl.takeOffAll()
            pass
        elif mission == "mission_land_all":
            missionControl.landAll()
            pass
        elif mission =="mission_takeoff":
            if target:
                missionControl.takeOffAgent(target=target)
                pass
        elif mission =="mission_land":
            if target:
                missionControl.landAgent(target=target)
                pass
        
if __name__ == "__main__":
    main()
    sys.exit(0)
