import os
import sys
from typing import List
import logging
import select

from threading import Thread
import numpy as np

from pycrazyswarm import Crazyswarm

from Agent import Agent
from MissionControl import MissionControl

def get_char():
    """Reads a char from the stdin.

    Returns:
        str: Read char.
    """
    ret_value = ""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        ret_value = sys.stdin.read(1)
    return ret_value

def watch_dog(agents: List[Agent]) -> None:
    """Runs on a thread. Checks the Redis 'emergency' channel and the input 'q'.
    Calls the function emergencyExit() if the need arises.

    Args:
        agents (List[Agent]): _description_
    """
    try:
        while True:
            key = get_char()

            if key == "q":
                emergency_exit(agents)

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt detected.")

def emergency_exit(agents: List[Agent]) -> None:
    """Kills the ROS server and turns off all the agents.
    """
    logging.info("Emergency! Exiting the program.")
    for agent in agents:
        agent.kill()
    sys.exit(-3)

def main() -> None:
    """Entry point for the ASRO software. Initializes the ROS server.
    Creates Agents and the Redis server which will be used by the WebApplication.
    
    Checks the Redis 'control' channel for any new mission requests.
    If a new request exists, hands over the control to MissionControl.
    """
    # Initialization
    agents: List[Agent]
    agents = []

    # Fix the issue where rospy disables the logging
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = "`rospack find rosgraph`/conf/python_logging.yaml"

    logging.info("Initializing Crasyswarm server")

    # Start the Crazyswarm server
    crazy_swarm = Crazyswarm(crazyflies_yaml="./crazyflies.yaml")

    logging.info("Creating instances of 'Agent' class.")

    # Add agents
    i = 0
    for agent in crazy_swarm.allcfs.crazyflies:
        agents.append(
            Agent(
                cf=agent,
                name=f"cf{agent.id}",
                idx=i
            )
        )
        i += 1
        logging.info(f"Created: cf{agent.id}")
        crazy_swarm.timeHelper.sleep(1.0)
    
    # Give the other agents' info to every agent
    for agent in agents:
        agent.setOtherAgents(agents)

    logging.info("Created all agents.")

    logging.info("Creating an instance of 'MissionControl'.")

    # Creating mission control    
    mission_control = MissionControl(
        agents=agents,
        crazyServer=crazy_swarm
    )
    
    # Start the watchdog
    watchdog_thread = Thread(target=watch_dog, args=[agents], daemon=True)
    watchdog_thread.start()

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
        sys.exit(0)

    elif "agent_trajectory_test" in sys.argv:
        mission_control.takeOffAgent(agents[0])
        mission_control.goToAgent(
            targetAgent=agents[0],
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
        mission_control.landAgent(agents[0])

    elif "takeoff_land_test" in sys.argv:
        for i in range(3):
            mission_control.takeOffAll()
            mission_control.landAll()
            crazy_swarm.timeHelper.sleep(3.0)

    elif "formation_test" in sys.argv:
        for i, agent in enumerate(agents):
            logging.info(f"Index: {i}, agent: {agent.getName()}")
            
        mission_control.takeOffAll()
        mission_control.takeFormation()
        mission_control.landAll()

    elif "rotation_test" in sys.argv:
        mission_control.takeOffAll()
        mission_control.takeFormation()
        mission_control.rotateSwarm(90.0)
        mission_control.rotateSwarm(-90.0)
        mission_control.rotateSwarm(-90.0)
        mission_control.rotateSwarm(90.0)
        mission_control.landAll()

    elif "swarm_trajectory_test" in sys.argv:
        mission_control.takeOffAll()
        mission_control.takeFormation()
        mission_control.goToSwarm(
            points=np.array(
                [
                    [3.0, 0.0],
                    [0.0, 0.0],
                    [-3.0, 0.0],
                    [0.0, 0.0]
                ]
            )
        )
        mission_control.landAll()

    elif "mission_one" in sys.argv:
        mission_control.takeOffAll()
        mission_control.takeFormation()
        mission_control.goToSwarm(np.array([[-2.0, -2.0]]))
        mission_control.rotateSwarm(-90.0)
        mission_control.goToSwarm(np.array([[-2.0, 2.0]]))
        mission_control.rotateSwarm(-90.0)
        mission_control.goToSwarm(np.array([[2.0, 2.0]]))
        mission_control.rotateSwarm(-90.0)
        mission_control.goToSwarm(np.array([[2.0, -2.0]]))
        mission_control.landAll()

    else:
        logging.info("Please specify the operation by giving an argument")

            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
    main()
    logging.info("All went according to the plan, good bye!")
    sys.exit(0)
