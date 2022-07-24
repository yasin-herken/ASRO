import os
import sys
from typing import List
import logging
import select

from threading import Thread
import numpy as np

import Settings
from Agent import Agent
from MissionControl import MissionControl
from pycrazyswarm import Crazyswarm

def get_char(block = False) -> str:
    """Gets a char from the stdin.

    Args:
        block (bool, optional): Whateher the thread is blocking or not. Defaults to False.

    Returns:
        str: Char.
    """
    ret_value = ""
    if block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        ret_value = sys.stdin.read(1)
    return ret_value

def watch_dog(agents: List[Agent]) -> None:
    """Runs on a thread. Checks the Redis 'emergency' channel and the input 'q'.
    Calls the function emergencyExit() if the need arises.

    Args:
        agents (List[Agent]): List of agents.
    """
    while True:
        key = get_char()

        if key == "q":
            emergency_exit(agents)

def emergency_exit(agents: List[Agent]) -> None:
    """Kills the ROS server and turns off all the agents.

    Args:
        agents (List[Agent]): List of agents.
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
    agent_count = 5

    created_agents: List[Agent]
    activa_agents: List[Agent]
    created_agents = []
    activa_agents = []
    

    # Fix the issue where rospy disables the logging
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = "`rospack find rosgraph`/conf/python_logging.yaml"

    logging.info("Initializing Crasyswarm server")

    # Start the Crazyswarm server
    crazy_swarm = Crazyswarm(crazyflies_yaml="./crazyflies.yaml")

    logging.info("Creating instances of 'Agent' class.")

    # Add agents
    for idx, agent in enumerate(crazy_swarm.allcfs.crazyflies):
        created_agents.append(
            Agent(
                cf=agent,
                name=f"cf{agent.id}",
                idx=idx
            )
        )

        logging.info(f"Created: cf{agent.id}")
        crazy_swarm.timeHelper.sleep(1.0)

    # List of all active agents    
    for i in range(agent_count):
        activa_agents.append(created_agents[i])
    
    # Give the all agents' info to every other agent
    for agent in created_agents:
        agent.setOtherAgents(activa_agents)

    logging.info(f"Created all agents.")

    # Log all agents' initialPos for debug
    for agent in activa_agents:
        logging.info(f"[{agent.getName()}] Initial position: {agent.getInitialPos().round(2)}")

    logging.info("Creating an instance of 'MissionControl'.")

    # Creating mission control    
    mission_control = MissionControl(
        agents=activa_agents,
        crazyServer=crazy_swarm
    )
    
    # Start the watchdog
    watchdog_thread = Thread(target=watch_dog, args=[activa_agents], daemon=True)
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
        mission_control.takeOffAgent(activa_agents[0], 0.5, 2.0)
        mission_control.goToAgent(
            targetAgent=activa_agents[0],
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
        mission_control.landAgent(activa_agents[0], 5.0)

    elif "takeoff_land_test" in sys.argv:
        for i in range(3):
            mission_control.takeOffAll(0.5, 2.0)
            mission_control.landAll(2.0)
            crazy_swarm.timeHelper.sleep(3.0)

    elif "formation_test" in sys.argv:
        for i, agent in enumerate(activa_agents):
            logging.info(f"Index: {i}, agent: {agent.getName()}")
            
        mission_control.takeOffAll(0.5, 2.0)
        mission_control.takeFormation(Settings.v(), 10.0)
        mission_control.landAll(5.0)

    elif "rotation_test" in sys.argv:
        mission_control.takeOffAll(0.5, 2.0)
        mission_control.takeFormation(Settings.pyramid(), 15.0)
        mission_control.rotateSwarm(90.0, 15.0)
        mission_control.rotateSwarm(-90.0, 15.0)
        mission_control.rotateSwarm(-90.0, 15.0)
        mission_control.rotateSwarm(90.0, 15.0)
        mission_control.landAll(5.0)

    elif "swarm_trajectory_test" in sys.argv:
        mission_control.takeOffAll(0.5, 3.0)
        mission_control.takeFormation(Settings.pyramid(), 15.0)
        mission_control.goToSwarm(np.array([[-3.0, -3.0, 0.5]]), 10.0)
        mission_control.goToSwarm(np.array([[-3.0, 3.0, 0.5]]), 10.0)
        mission_control.rotateSwarm(-90.0, 6.0)
        mission_control.goToSwarm(np.array([[3.0, 3.0, 0.5]]), 10.0)
        mission_control.rotateSwarm(-90.0, 6.0)
        mission_control.goToSwarm(np.array([[3.0, -3.0, 0.5]]), 10.0)
        mission_control.landAll(5.0)
    
    elif "mission_one" in sys.argv:
        mission_control.takeOffAll(0.5, 3.0)
        mission_control.takeFormation(Settings.pyramid(), 15.0)
        mission_control.goToSwarm(np.array([[0.0, 3.0, 0.5]]), 15.0)
        mission_control.landAll(5.0)
    
    elif "mission_two" in sys.argv:
        mission_control.takeOffAll(0.5, 3.0)
        mission_control.takeFormation(Settings.pyramid(), 15.0)
        mission_control.goToSwarm(np.array([[0.0, 3.0, 0.5]]), 15.0)
        inactive_agents = list(set(created_agents) - set(activa_agents))
        mission_control.swapSwarmAgents(activa_agents[-2:], inactive_agents, [10.0, 10.0, 10.0, 10.0])
        mission_control.goToSwarm(np.array([[-3.0, 3.0, 0.5]]), 15.0)
        mission_control.rotateSwarm(90.0, 6.0)
        mission_control.goToSwarm(np.array([[-3.0, -3.0, 0.5]]), 15.0)
        mission_control.landAll(5.0)

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

        sys.exit(0)

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%d/%m/%Y %H:%M:%S'
    )
    main()
    logging.info("All went according to the plan, good bye!")
    sys.exit(0)
