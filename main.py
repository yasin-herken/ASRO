"Control whole project here and decide which mission is run"
import os
import sys
from typing import List
import logging
import select
import time
from threading import Thread
import numpy as np
import cflib.crtp
from cflib.utils import uri_helper
from torch import true_divide
import settings
from agent import Agent
from mission_control import MissionControl
import rospy
from custom_msg.msg import general_parameters

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
    Calls the function emergency_exit() if the need arises.
    """

    while True:
        key = get_char()

        if key == "q":
            emergency_exit(agents)

def emergency_exit(agents: List[Agent]) -> None:
    """Kills the ROS server and turns off all the agents.
    """
    logging.info("Emergency! Exiting the program.")
    for agent in agents:
        agent.kill()
    sys.exit(-3)
logging.basicConfig(level=logging.ERROR)
def main() -> None:
    """Entry point for the ASRO software. Initializes the ROS server.
    Creates Agents and the Redis server which will be used by the WebApplication.
    
    Checks the Redis 'control' channel for any new mission requests.
    If a new request exists, hands over the control to MissionControl.
    """
    # Initialization
    agent_count = 1
    created_agents: List[Agent]
    activated_agents: List[Agent]
    created_agents = []
    activated_agents = []
    uri_list = []
    # Agent URIs
    uri_list.append(uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7'))
    # Fix the issue where rospy disables the logging
    #os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = "`rospack find rosgraph`/conf/python_logging.yaml"


    # Start the Crazyswarm server
    cflib.crtp.init_drivers()
    
    print("CFlib created")
    rospy.init_node('position',anonymous=True)
    team_name = "YILDIZLARARASI"
    topic_name = "general_parameters/" +team_name 
    pub_stabilizer = rospy.Publisher(str(topic_name), general_parameters, queue_size=10) 
    print("Publisher")
    # Add agents
    for idx,uri in enumerate(uri_list):
        created_agents.append(
            Agent(
                uri=uri,
                name=str(int(uri[-2:], 16)),
                idx=idx,
                pub_stabilizer= pub_stabilizer,
            )
        )
    print("Agent Created")
    time.sleep(5.0)
    # List of all active agents

    for i in range(agent_count):
        activated_agents.append(created_agents[i])
    # Give the all agents' info to every other agent
    for agent in created_agents:
        agent.set_other_agents(activated_agents)

    # Creating mission control    
    mission_control = MissionControl(
        agents = activated_agents
    )
    # Start the watchdog
    watchdog_thread = Thread(target=watch_dog, args=[activated_agents], daemon=True)
    watchdog_thread.start()

    print("All agent ready to fly")

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
        mission_control.take_off_agent(activated_agents[0], 0.5, 2.0)
        mission_control.goto_agent(
            target_agent=activated_agents[0],
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
        mission_control.land_agent(activated_agents[0], 5.0)

    elif "takeoff_land_test" in sys.argv:
        print("-----------------------------------------------------")
        mission_control.take_off_all(0.5, 5.0)
        mission_control.land_all(2.0)

    elif "formation_test" in sys.argv:
        for i, agent in enumerate(activated_agents):
            logging.info(f"Index: {i}, agent: {agent.get_name()}")
        mission_control.take_off_all(0.5, 2.0)
        mission_control.take_formation(settings.v_shape(), 10.0)
        mission_control.land_all(5.0)

    elif "rotation_test" in sys.argv:
        mission_control.take_off_all(0.5, 2.0)
        mission_control.take_formation(settings.pyramid(), 15.0)
        mission_control.rotate_swarm(90.0, 15.0)
        mission_control.rotate_swarm(-90.0, 15.0)
        mission_control.rotate_swarm(-90.0, 15.0)
        mission_control.rotate_swarm(90.0, 15.0)
        mission_control.land_all(5.0)

    elif "swarm_trajectory_test" in sys.argv:
        mission_control.take_off_all(0.5, 3.0)
        mission_control.take_formation(settings.pyramid(), 15.0)
        mission_control.goto_swarm(np.array([[-3.0, -3.0, 0.5]]), 10.0)
        mission_control.goto_swarm(np.array([[-3.0, 3.0, 0.5]]), 10.0)
        mission_control.rotate_swarm(-90.0, 6.0)
        mission_control.goto_swarm(np.array([[3.0, 3.0, 0.5]]), 10.0)
        mission_control.rotate_swarm(-90.0, 6.0)
        mission_control.goto_swarm(np.array([[3.0, -3.0, 0.5]]), 10.0)
        mission_control.land_all(5.0)
    elif "mission_one" in sys.argv:
        mission_control.take_off_all(0.5, 3.0)
        mission_control.take_formation(settings.pyramid(), 15.0)
        mission_control.goto_swarm(np.array([[0.0, 3.0, 0.5]]), 15.0)
        mission_control.land_all(5.0)
    elif "mission_two" in sys.argv:
        mission_control.take_off_all(0.5, 3.0)
        mission_control.take_formation(settings.pyramid(), 15.0)
        mission_control.goto_swarm(np.array([[0.0, 3.0, 0.5]]), 15.0)
        inactive_agents = list(set(created_agents) - set(activated_agents))
        mission_control.swap_swarm_agents(activated_agents[-2:], inactive_agents, [10.0, 10.0, 10.0, 10.0])
        mission_control.goto_swarm(np.array([[-3.0, 3.0, 0.5]]), 15.0)
        mission_control.rotate_swarm(90.0, 6.0)
        mission_control.goto_swarm(np.array([[-3.0, -3.0, 0.5]]), 15.0)
        mission_control.land_all(5.0)
    elif "mission_three" in sys.argv:
        mission_control.take_off_all(0.5, 3.0)
        mission_control.take_formation(settings.v_shape(), 15.0)
        inactive_agents = list(set(created_agents) - set(activated_agents))
        mission_control.load_obstacles(inactive_agents, 1.0)
        mission_control.goto_swarm(np.array([[3.0, 0.0, 0.5]]), 15.0)
        mission_control.land_all(5.0)
    elif "mission_four" in sys.argv:
        mission_control.take_off_all(1.0,10.0)
        print("here")
        mission_control.take_formation(settings.triangle(),5.0)
        mission_control.goto_swarm(np.array([-1.0,-1.0,1.0]),10.0)
        mission_control.land_all(3.0)
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