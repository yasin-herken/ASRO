import os
import sys
from typing import List
import logging
import select
import numpy as np

from pycrazyswarm import Crazyswarm

from Agent import Agent
from MissionControl import MissionControl
from threading import Thread

def getChar(block = False):
    if block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)

def _watchDog(agents: List[Agent]) -> None:
    """Runs on a thread. Checks the Redis 'emergency' channel and the input 'q'.
    Calls the function emergencyExit() if the need arises.
    """
    try:
        while True:
            key = getChar()

            if key == "q":
                emergencyExit(agents)

    except KeyboardInterrupt as e:
        logging.info("Keyboard interrupt detected.")
    except Exception as e:
        logging.info(f"An exception occured.\n {e.with_traceback()}")

def emergencyExit(agents: List[Agent]) -> None:
    """Kills the ROS server and turns off all the agents.
    """
    logging.info("Emergency! Exiting the program.")
    for agent in agents:
        agent.kill()
    os._exit(-3)

def main() -> None:
    """Entry point for the ASRO software. Initializes the ROS server.
    Creates Agents and the Redis server which will be used by the WebApplication.
    
    Checks the Redis 'control' channel for any new mission requests.
    If a new request exists, hands over the control to MissionControl.
    """
    # Initialization
    agents = []
    cfIds = []

    logging.info("Initializing Crasyswarm server")

    # Start the Crazyswarm server
    crazySwarm = Crazyswarm(crazyflies_yaml="./crazyflies.yaml")

    logging.info("Creating instances of 'Agent' class.")
    
    # Geting the agent ids
    for cfId in crazySwarm.allcfs.crazyfliesById:
        cfIds.append(cfId)

    # Add agents
    for agent in crazySwarm.allcfs.crazyflies:
        agents.append(
            Agent(
                cf=agent,
                initialPos=agent.initialPosition,
                name=f"cf{agent.id}"
            )
        )
        logging.info(f"Created: cf{agent.id}")

    logging.info(f"Created all agents.")
    logging.info("Creating an instance of 'MissionControl'.")

    # Creating mission control    
    missionControl = MissionControl(
        agents=agents,
        crazyServer=crazySwarm
    )
    
    # Start the watchdog
    watchdogThread = Thread(target=_watchDog, args=[agents])
    watchdogThread.start()

    logging.info("All ready! Listening... Press 'q' to exit.")

    # Update the agents
    for agent in agents:
        agent.update(agents)
    crazySwarm.timeHelper.sleep(1 / 100)

    if "trajectory_test" in sys.argv:
        missionControl.takeOffAgent(agents[0])
        missionControl.goToAgent(
            agent=agents[0],
            points=np.array(
                [
                    [0.5, 0.0, 0.5],
                    [0.0, 0.0, 0.5],
                    [-0.5, 0.0, 0.5],
                    [0.0, 0.0, 0.5],
                    [0.0, 0.5, 0.5],
                    [0.0, 0.0, 0.5],
                    [0.0, -0.5, 0.5],
                    [0.0, 0.0, 0.5]
                ]
            )
        )
        missionControl.landAgent(agents[0])

    elif "takeoff_land_test" in sys.argv:
        for i in range(3):
            missionControl.takeOffAll()
            crazySwarm.timeHelper.sleep(1)
            missionControl.landAll()
            crazySwarm.timeHelper.sleep(1)

    elif "formation_test" in sys.argv:
        for i, agent in enumerate(agents):
            logging.info(f"Index: {i}, agent: {agent.getName()}")
            
        missionControl.takeOffAll()
        crazySwarm.timeHelper.sleep(5)
        missionControl.testFormation()
        crazySwarm.timeHelper.sleep(5)
        missionControl.landAll()

    else:
        logging.info("Please specify the operation by giving an argument")
    

            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
    main()
    logging.info("All went according to the plan, good bye!")
    os._exit(0)
