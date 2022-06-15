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
    agents = []

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
                name=f"cf{1}"
            ))

    logging.info(f"Created agents.")
    logging.info("Creating an instance of 'MissionControl'.")

    # Creating mission control    
    missionControl = MissionControl(
        rospyRate=rospyRate,
        agents=agents
    )
    
    # Start the watchdog
    watchdogThread = Thread(target=_watchDog, args=[agents, rospyRate])
    watchdogThread.start()

    i = 0

    logging.info("All ready! Listening... Press 'q' to exit.")

    if "trajectory_test" in sys.argv:
        missionControl.takeOffAgent(target=agent[0])
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
            maxVel=0.5
        )
        missionControl.landAgent(target=agent[0])
    elif "takeoff_land_test" in sys.argv:
        missionControl.takeOffAll()
        rospy.sleep(5)
        missionControl.landAll(agent[0])
        rospy.sleep(5)
    elif "formation_test" in sys.argv:
        for i, agent in enumerate(agents):
            logging.info(f"Index: {i}, agent: {agent.getName()}")
            
        missionControl.takeOffAll()
        rospy.sleep(5)
        missionControl.testFormation()
        rospy.sleep(5)
        missionControl.landAll()
            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
    main()
    sys.exit(0)
