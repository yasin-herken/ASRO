import os
import sys
from typing import List
from cupshelpers import missingExecutables
import redis
import logging
import json
import select
import rospy
import numpy as np
import time
from sim_cf import crazyflie
from Agent import Agent
from MissionControl import MissionControl, Point
from threading import Thread

def getChar(block = False):
    if block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)

def _watchDog(agents: List[Agent], rospyRate: rospy.Rate) -> None:
    """Runs on a thread. Checks the Redis 'emergency' channel and the input 'q'.
    Calls the function emergencyExit() if the need arises.
    """
    try:
        while not rospy.is_shutdown():
            key = getChar()

            if key == "q":
                emergencyExit(agents)

            # Update the agents
            for agent in agents:
                agent.update(agents)
            rospyRate.sleep()

    except KeyboardInterrupt as e:
        logging.info("Keyboard interrupt detected.")
    except Exception as e:
        logging.info(f"An exception occured.\n {e.with_traceback()}")

def emergencyExit(agents: List[Agent]) -> None:
    """Kills the ROS server and turns off all the agents.
    """
    logging.info("Emergency! Exiting the program.")
    for agent in agents:
        agent.landAsync()
        agent.kill()
    rospy.signal_shutdown("Emergency exit.")
    os._exit(-3)

def main() -> None:
    """Entry point for the ASRO software. Initializes the ROS server.
    Creates Agents and the Redis server which will be used by the WebApplication.
    
    Checks the Redis 'control' channel for any new mission requests.
    If a new request exists, hands over the control to MissionControl.
    """
    # Initialization
    i = 0
    agentCount = 6
    cfIds = []
    agents = []
    names = []
    addresses = []
    statuses = []

    # Fix the issue where rospy disables the logging
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = "`rospack find rosgraph`/conf/python_logging.yaml"

    logging.info("Initializing the ROS Node")

    # Start the ROS server
    rospy.init_node('ASRO')
    rospyRate = rospy.Rate(100) # Hz

    logging.info("Creating instances of 'Agent' class.")

    # Add agents
    agents.append(Agent(
                cf=crazyflie.Crazyflie(f"cf{1}", f"/cf{1}"),
                initialPos= np.array([0.0, -1.0, 0.0]),
                name=f"cf{1}",
                address=f"radio:/{1}//80/2M - Unknown"
            ))
    names.append("cf1")


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
        rospyRate=rospyRate,
        agents=agents,
        redisClient=redisClient
    )
    
    # Start the watchdog
    watchdogThread = Thread(target=_watchDog, args=[agents, rospyRate])
    watchdogThread.start()

    i = 0

    logging.info("All ready! Listening... Press 'q' to exit.")
    while True:
        # Read incoming messages from redis
        try:
            msg = redisSub.get_message()
        except KeyboardInterrupt as e:
            logging.info("Keyboard interrupt detected.")
            emergencyExit()

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
            elif "position_test" in sys.argv:
                mission = "mission_position_test"
                sys.argv.remove("position_test")
            elif "local_rotation_test" in sys.argv:
                mission = "mission_local_rotation_test"
                sys.argv.remove("local_rotation_test")

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
            missionControl.takeOffAgent(target=names[0])
            missionControl.goToAgent(
                target="cf1",
                points=[
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
                ],
                maxVel=0.75
            )
            missionControl.landAgent(target=names[0])
        
        elif mission == "mission_takeoff_land_test":
            i = 0
            while i < 3:
                i += 1
                missionControl.takeOffAgent(names[0])
                missionControl.landAgent(names[0])
                rospy.sleep(3)

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
            time.sleep(5.0)
            missionControl.testFormation()
            rospy.sleep(5)
            missionControl.landAll()
        
        elif mission == "mission_position_test":
            missionControl.testPosition()

        elif mission == "mission_local_rotation_test":
            # TODO: To it in a loop
            missionControl.takeOffAgent(names[0])
        
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
    main()
    sys.exit(0)
