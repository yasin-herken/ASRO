import os
import sys
import redis
import logging
import json
import select
import numpy as np

from Agent import Agent
from MissionControl import MissionControl, Point
from pycrazyswarm import Crazyswarm
from threading import Thread

def getChar(block = False):
    if block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)

def _watchDog(server: Crazyswarm) -> None:
    """Runs on a thread. Checks the Redis 'emergency' channel and the input 'q'.
    Calls the function emergencyExit() if the need arises.

    Args:
        server (Crazyswarm): To be used when calling emergencyExit().
    """
    try:
        while True:
            key = getChar()

            if key == "q":
                emergencyExit(server)
    except KeyboardInterrupt as e:
        logging.info("Keyboard interrupt detected.")
    except Exception as e:
        logging.info(f"An exception occured.\n {e.with_traceback()}")

def emergencyExit(server: Crazyswarm) -> None:
    """Kills the Crazyswarm server and turns off all the agents.

    Args:
        server (Crazyswarm): Server to be killed.
    """
    logging.info("Emergency! Exiting the program.")
    os._exit(-3)

def main() -> None:
    """Entry point for the ASRO software. Initializes the Crazyswarm server.
    Creates Agents and the Redis server which will be used by the WebApplication.
    
    Checks the Redis 'control' channel for any new mission requests.
    If a new request exists, hands over the control to MissionControl.
    """
    # Initialization
    i = 0
    cfIds = []
    agents = []
    names = []
    addresses = []
    statuses = []
    configFile = "./crazyflies.yaml"

    logging.info(f"Reading config file '{configFile}'.")

    # Start the crazyswarm/ros server
    crazySwarm = Crazyswarm(crazyflies_yaml=configFile)

    # Geting the agent ids
    for id in crazySwarm.allcfs.crazyfliesById:
        cfIds.append(id)

    logging.info("Creating instances of 'Agent' class.")

    # Creating the agents
    for agent in crazySwarm.allcfs.crazyflies:
        agents.append(
            Agent(
                cf=agent,
                name=f"Agent {cfIds[i]}",
                address=f"radio:/{cfIds[i]}/125/2M"
            )
        )
        names.append(f"Agent {cfIds[i]}")
        addresses.append(f"radio:/{cfIds[i]}/125/2M")
        i+=1

    logging.info(f"Created agents '{names}'.")

    # Initializing redis
    logging.info("Initializing Redis.")
    redisClient = redis.Redis()
    redisClient.set("agents", json.dumps({"names": names, "addresses": addresses}))
    redisSub = redisClient.pubsub()
    redisSub.subscribe(["requests"])
    
    logging.info("Creating an instance of 'MissionControl'.")

    # Creating mission control    
    missionControl = MissionControl(
        crazySwarm=crazySwarm,
        agents=agents,
        redisClient=redisClient
    )
    
    # Start the watchdog
    watchdogThread = Thread(target=_watchDog, args=[crazySwarm])
    watchdogThread.start()

    i = 0

    logging.info("All ready! Listening... Press 'q' to exit.")
    while True:
        # Read incoming messages from redis
        try:
            msg = redisSub.get_message()
        except KeyboardInterrupt as e:
            logging.info("Keyboard interrupt detected.")
            emergencyExit(crazySwarm)

        mission = ""
        target = ""
        
        # Parse the message
        if msg is not None and msg["type"] == "message":
            parsed = json.loads(msg['data'])

            mission = parsed.get("mission", None)
            if mission:
                logging.info(f"Incoming misson request: {mission}")

            target = parsed.get("target", None)
            if target:
                logging.info(f"Incoming target: {target}")
        else:
            if "trajectory_test" in sys.argv:
                mission = "mission_trajectory_test"
                sys.argv.remove("trajectory_test")
            elif "takeoff_land_test" in sys.argv:
                mission = "mission_takeoff_land_test"
                sys.argv.remove("takeoff_land_test")
            elif "takeoff_test" in sys.argv:
                mission = "mission_takeoff_test"
                sys.argv.remove("takeoff_test")
            elif "takeoff_all_test" in sys.argv:
                mission = "mission_takeoff_all_test"
                sys.argv.remove("takeoff_all_test")
            elif "land_test" in sys.argv:
                mission = "mission_land_test"
                sys.argv.remove("land_test")
            elif "land_all_test" in sys.argv:
                mission = "mission_land_all_test"
                sys.argv.remove("land_all_test")
            elif "formation_test" in sys.argv:
                mission = "mission_formation_test"
                sys.argv.remove("formation_test")

        # Launch a mission if a message exists
        if mission == "mission_takeoff_all":
            missionControl.takeOffAll()

        elif mission == "mission_land_all":
            missionControl.landAll()

        elif mission == "mission_takeoff" and target:
            missionControl.takeOffAgent(target=target)

        elif mission == "mission_land" and target:
            missionControl.landAgent(target=target)

        elif mission == "mission_one":
            missionControl.missionOne()
        
        elif mission == "mission_trajectory_test":
            # TODO: To it in a loop
            missionControl.goToAgent(
                names[0],
                [
                    Point(0.0, 0.0, 0.5, False),
                    Point(0.5, 0.0, 0.5, False),
                    Point(0.0, 0.0, 0.5, False),
                    Point(-0.5, 0.0, 0.5, False),
                    Point(0.0, 0.0, 0.5, False),
                    Point(0.0, 0.5, 0.5, False),
                    Point(0.0, 0.0, 0.5, False),
                    Point(0.0, -0.5, 0.5, False),
                    Point(0.0, 0.0, 0.5, False),
                    Point(0.0, 0.0, 1.0, False),
                    Point(0.0, 0.0, 0.5, False),
                    Point(0.0, 0.0, 0.05, False),
                ],
                1.0
            )
            emergencyExit(crazySwarm)
        
        elif mission == "mission_takeoff_land_test":
            i = 0
            while i < 3:
                i += 1
                missionControl.takeOffAgent(names[0])
                missionControl.landAgent(names[0])

        elif mission == "mission_takeoff_test":
            # TODO: To it in a loop
            missionControl.takeOffAgent(names[0])

        elif mission == "mission_takeoff_all_test":
            # TODO: To it in a loop
            missionControl.takeOffAll()
        
        elif mission == "mission_land_test":
            # TODO: To it in a loop
            missionControl.landAgent(names[0])

        elif mission == "mission_land_all_test":
            # TODO: To it in a loop
            missionControl.landAll()
        elif mission == "mission_formation_test":
            for i, agent in enumerate(agents):
                logging.info(f"Index: {i}, agent: {agent.getName()}")
                
            missionControl.takeOffAll()
            missionControl.testFormation()

        crazySwarm.timeHelper.sleep(1 / 100)
        
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
    main()
    sys.exit(0)
