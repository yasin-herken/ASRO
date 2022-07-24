import os
import sys
from typing import List
import logging
import select
import numpy as np
from pycrazyswarm import Crazyswarm

import Settings
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
    agentCount = 5

    createdAgents: List[Agent]
    activeAgents: List[Agent]
    createdAgents = []
    activeAgents = []
    

    # Fix the issue where rospy disables the logging
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = "`rospack find rosgraph`/conf/python_logging.yaml"

    logging.info("Initializing Crasyswarm server")

    # Start the Crazyswarm server
    crazySwarm = Crazyswarm(crazyflies_yaml="./crazyflies.yaml")

    logging.info("Creating instances of 'Agent' class.")

    # Add agents
    for idx, agent in enumerate(crazySwarm.allcfs.crazyflies):
        createdAgents.append(
            Agent(
                cf=agent,
                name=f"cf{agent.id}",
                idx=idx
            )
        )

        logging.info(f"Created: cf{agent.id}")
        crazySwarm.timeHelper.sleep(1.0)

    # List of all active agents    
    for i in range(agentCount):
        activeAgents.append(createdAgents[i])
    
    # Give the all agents' info to every other agent
    for agent in createdAgents:
        agent.setOtherAgents(activeAgents)

    logging.info(f"Created all agents.")

    # Log all agents' initialPos for debug
    for agent in activeAgents:
        logging.info(f"[{agent.getName()}] Initial position: {agent.getInitialPos().round(2)}")

    logging.info("Creating an instance of 'MissionControl'.")

    # Creating mission control    
    missionControl = MissionControl(
        agents=activeAgents,
        crazyServer=crazySwarm
    )
    
    # Start the watchdog
    watchdogThread = Thread(target=_watchDog, args=[activeAgents], daemon=True)
    watchdogThread.start()

    logging.info("All ready! Listening... Press 'q' to exit.")

    if "help" in sys.argv:
        print("\nAvailable parameters")
        print("---------------------")
        print("agent_trajectory_test")
        print("takeoff_land_test")
        print("formation_test")
        print("rotation_test")
        print("swarm_trajectory_test")
        print("---------------------\n")
        os._exit(0)

    elif "agent_trajectory_test" in sys.argv:
        missionControl.takeOffAgent(activeAgents[0], 0.5, 2.0)
        missionControl.goToAgent(
            targetAgent=activeAgents[0],
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
            ),
            duration=5.0
        )
        missionControl.landAgent(activeAgents[0], 5.0)

    elif "takeoff_land_test" in sys.argv:
        for i in range(3):
            missionControl.takeOffAll(0.5, 2.0)
            missionControl.landAll(2.0)
            crazySwarm.timeHelper.sleep(3.0)

    elif "formation_test" in sys.argv:
        for i, agent in enumerate(activeAgents):
            logging.info(f"Index: {i}, agent: {agent.getName()}")
            
        missionControl.takeOffAll(0.5, 2.0)
        missionControl.takeFormation(Settings.v(), 10.0)
        missionControl.landAll(5.0)

    elif "rotation_test" in sys.argv:
        missionControl.takeOffAll(0.5, 2.0)
        missionControl.takeFormation(Settings.pyramid(), 15.0)
        missionControl.rotateSwarm(90.0, 15.0)
        missionControl.rotateSwarm(-90.0, 15.0)
        missionControl.rotateSwarm(-90.0, 15.0)
        missionControl.rotateSwarm(90.0, 15.0)
        missionControl.landAll(5.0)

    elif "swarm_trajectory_test" in sys.argv:
        missionControl.takeOffAll(0.5, 3.0)
        missionControl.takeFormation(Settings.pyramid(), 15.0)
        missionControl.goToSwarm(np.array([[-3.0, -3.0, 0.5]]), 10.0)
        missionControl.goToSwarm(np.array([[-3.0, 3.0, 0.5]]), 10.0)
        missionControl.rotateSwarm(-90.0, 6.0)
        missionControl.goToSwarm(np.array([[3.0, 3.0, 0.5]]), 10.0)
        missionControl.rotateSwarm(-90.0, 6.0)
        missionControl.goToSwarm(np.array([[3.0, -3.0, 0.5]]), 10.0)
        missionControl.landAll(5.0)
    
    elif "mission_one" in sys.argv:
        missionControl.takeOffAll(0.5, 3.0)
        missionControl.takeFormation(Settings.pyramid(), 15.0)
        missionControl.goToSwarm(np.array([[0.0, 3.0, 0.5]]), 15.0)
        missionControl.landAll(5.0)
    
    elif "mission_two" in sys.argv:
        missionControl.takeOffAll(0.5, 3.0)
        missionControl.takeFormation(Settings.pyramid(), 15.0)
        missionControl.goToSwarm(np.array([[0.0, 3.0, 0.5]]), 15.0)
        inactiveAgents = list(set(createdAgents) - set(activeAgents))
        missionControl.swapSwarmAgents(activeAgents[-2:], inactiveAgents, [10.0, 10.0, 10.0, 10.0])
        missionControl.goToSwarm(np.array([[-3.0, 3.0, 0.5]]), 15.0)
        missionControl.rotateSwarm(90.0, 6.0)
        missionControl.goToSwarm(np.array([[-3.0, -3.0, 0.5]]), 15.0)
        missionControl.landAll(5.0)

    else:
        logging.info("Please specify the operation by giving an argument")

        print("\nAvailable parameters")
        print("---------------------")
        print("agent_trajectory_test")
        print("takeoff_land_test")
        print("formation_test")
        print("rotation_test")
        print("swarm_trajectory_test")
        print("---------------------\n")

        os._exit(0)

            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
    main()
    logging.info("All went according to the plan, good bye!")
    os._exit(0)
