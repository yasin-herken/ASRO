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

    crazySwarm = Crazyswarm(crazyflies_yaml="./crazyflies.yaml")
    
    agents = []
    i = 1
    for agent in crazySwarm.allcfs.crazyflies:
        agents.append(
            Agent(
                cf=agent,
                name="cf"+str(i),
                address="random_ass_string"
            )
        )
        i+=1
    missionControl = MissionControl(
        crazySwarm=crazySwarm,
        agents=agents,
        redisClient=redis.Redis
    )
    missionControl._syncRedis()
    while True:
        # Read incoming messages from redis
        
        # Parse the message
    #pass
    
        # Launch a mission if a message exists
        missionControl.missionZero()
        pass
        # Update 'pyrazyswarm'
    #pass

if __name__ == "__main__":
    main()
    sys.exit(0)
