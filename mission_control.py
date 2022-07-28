"Control mission for each agents"
import logging
import time
from typing import List
import numpy as np
import settings
from agent import Agent

class MissionControl:
    """Handles the agent operations depending on the mission on hand.
    """
    __agents: List[Agent]
    
    __swarm_heading: np.ndarray
    __swarm_desired_heading: np.ndarray
    __rotation_angle: float

    def __init__(self, agents: List[Agent]):
        """Initialize the MissionControl.

        Args:
            agents (List[Agent]): Agents to be operated.
        """
        self.__agents = agents

        self.__rotation_angle = 0.0
        self.__swarm_heading = np.array([0.001, 0.001, 0.0])
        self.__swarm_desired_heading = np.array([0.0, 1.0, 0.0])
    def __update(self) -> bool:
        """Update the agents

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        # Synchronize swarm info

        # Update swarm info
        swarm_center = np.array([0.0, 0.0, 0.0])

        for agent in self.__agents:
            swarm_center += agent.get_pos()
        swarm_center /= len(self.__agents)

        front_agent = self.__agents[0]
        dist_diff = front_agent.get_pos() - swarm_center

        self.__swarm_heading = dist_diff / np.linalg.norm(dist_diff)
        angle_diff = settings.angle_between(self.__swarm_heading, self.__swarm_desired_heading)

        # Determine rotation direction
        # 1.0 for counter-clockwise -1.0 for counterwise             
        if (0.5 <= angle_diff):
            # print(angle_diff)
            rot_dir = 1.0

            x_1 = self.__swarm_heading[0]
            y_1 = self.__swarm_heading[1]

            x_2 = self.__swarm_desired_heading[0]
            y_2 = self.__swarm_desired_heading[1]

            temp_val = x_1 * y_2 - x_2 * y_1

            if temp_val <= 0.0:
                rot_dir = -1.0

            self.__rotation_angle += rot_dir * 0.05

            for agent in self.__agents:
                agent.set_rotation_angle(self.__rotation_angle)

        return True
    def __validate_formation_matrix(self, formation_matrix: np.ndarray) -> bool:
        rows_count = len(formation_matrix)
        is_valid = True
        ret_value = True
        
        for i in range(rows_count):
            columns_count = len(formation_matrix[i])
            if rows_count != columns_count:
                is_valid = False
        
        if not is_valid:
            logging.info("Formatin matrix is invalid. Rows and columns count do not match")
            ret_value = False

        if len(self.__agents) != rows_count:
            logging.info(f"Agent count and formation_matrix does not match. Agents: {len(self.__agents)}, formation_matrix: {rows_count}x{rows_count}")
            ret_value = False

        return ret_value

    def take_formation(self, formation_matrix: np.ndarray, duration: float) -> bool:
        """Takes the Agents into the specified formation.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        ret_value = False
        logging.info("Starting mission take_formation.")

        # Check if formation_matrix matches the agent count
        if not self.__validate_formation_matrix(formation_matrix):
            logging.info("Aborting!")
            return False

        # Activate and give the formation parameters
        for agent in self.__agents:
            agent.set_formation_matrix(formation_matrix)

        # Activate the formationControl and swarming
        for agent in self.__agents:
            agent.set_trajectory_active(False)
            agent.set_avoidance_active(True)
            agent.set_formation_active(True)
            agent.set_swarming(True)

        # Wait for the formation to happen
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= duration:
            t2_val = time.perf_counter()

        for agent in self.__agents:
            agent.set_formation_const(1.25)

        # Print the distances between
        for agent1 in self.__agents:
            for agent2 in self.__agents:
                if agent1 is not agent2:
                    print(f"{agent1.get_name()} {agent2.get_name()} -> {round(settings.get_distance(agent1.get_pos(), agent2.get_pos()), 2)}")
        logging.info(f"Ending mission take_formation with success.")

        return ret_value

    def swap_swarm_agents(self, target_agents: List[Agent], new_agents: List[Agent], durations: List[float]) -> bool:
        """_summary_

        Args:
            target_agents (List[Agent]): _description_
            new_agents (List[Agent]): _description_
            duration (float): _description_

        Returns:
            bool: _description_
        """
        ret_value = False
        logging.info("Starting mission swap_swarm_agents.")

        if len(target_agents) != len(new_agents):
            logging.info("Size of target_agents and new_agents are not the same! Aborting")
            return False

        if len(durations) != 4:
            logging.info("Size of durations is not 4! Aborting")
            return False

        # Stop formation forces
        for agent in self.__agents:
            agent.set_formation_active(False)
            agent.set_swarming(False)
            pos = agent.get_pos()
            agent.set_target_point(pos)

        # Prepare the targetAgents
        poses = []
        for agent in target_agents:
            poses.append(agent.get_pos())
            agent.set_trajectory_active(True)

        # Move the target_agents back to their spawn point
        for agent in target_agents:
            cur_pos = agent.get_pos()
            initial_pos = agent.get_initial_pos()
            initial_pos[2] = cur_pos[2]
            agent.set_target_point(initial_pos)

        # Wait for agents to go
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= durations[0]:
            t2_val = time.perf_counter()

        # Land the target_agents
        for agent in target_agents:
            agent.land()

        # Wait for target_agents to land
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= durations[1]:
            t2_val = time.perf_counter()

        # Remove target_agents from the agents list
        for agent in target_agents:
            self.__agents.remove(agent)

        # Take off the new_agents
        for agent in new_agents:
            agent.take_off(0.50)
            agent.set_avoidance_active(True)
            agent.set_trajectory_active(True)
        
        # Wait for new_agents to take_off
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= durations[2]:
            t2_val = time.perf_counter()

        # Move new_agents to their positions in the swarm
        for idx, agent in enumerate(new_agents):
            agent.set_target_point(poses[idx])

        # Wait for new_agents to move
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= durations[3]:
            t2_val = time.perf_counter()

        # Finalize the new_agents
        for idx, agent in enumerate(new_agents):
            # Add them to the list
            self.__agents.append(agent)

            # Give them the formation_matrix
            agent.set_formation_matrix(target_agents[idx].get_formation_matrix())

            # Give them the rotationAngle
            agent.set_rotation_angle(target_agents[idx].get_rotation_angle())

            # Give them the formationConst
            agent.set_formation_const(target_agents[idx].get_formation_const())

            # Give them the swarm desiredHeding
            agent.set_swarm_desired_heading(target_agents[idx].get_swarm_desired_heading())

            # Swap the indexes
            temp = target_agents[idx].get_index()
            target_agents[idx].set_index(agent.get_index())
            agent.set_index(temp)

        # Activate formation forces
        for agent in self.__agents:
            agent.set_formation_active(True)
            agent.set_swarming(True)

        logging.info(f"Ending mission swap_swarm_agents with success.")

        ret_value = True

        return ret_value
    def load_obstacles(self, obstacles: List[Agent], duration) -> bool:
        """Takes off the agent.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info(f"Starting mission load_obstacles. Obstacle count: {len(obstacles)}")
        ret_value = False

        for obstacle in obstacles:
            obstacle.set_is_static(True)

        for agent in self.__agents:
            agent.set_obstacles(obstacles)
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= duration:
            self.__update()
            t2_val = time.perf_counter()


        logging.info(f"Ending mission load_obstacles with success.")

        agent.set_trajectory_active(False)

        return ret_value

    def take_off_agent(self, agent: Agent, height: float, duration: float) -> bool:
        """Takes off the agent.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission take_off_agent.")
        ret_value = False

        agent.take_off(height)
        agent.set_trajectory_active(True)

        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= duration:
            t2_val = time.perf_counter()


        logging.info(f"Ending mission take_off_agent with success. Current height: {round(agent.get_pos()[2], 2)}")

        agent.set_trajectory_active(False)

        return ret_value
    
    def take_off_all(self, height: float, duration: float) -> bool:
        """Takes off all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission take_off_all.")
        ret_value = False

        for agent in self.__agents:
            ret_value = agent.take_off(height)
            agent.set_trajectory_active(True)
        time.sleep(duration)
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        '''while t2_val - t1_val <= duration:
            t2_val = time.perf_counter()'''


        logging.info(f"Ending mission take_off_all with success. Total target count: {len(self.__agents)}")

        #agent.set_trajectory_active(False)

        return ret_value

    def land_agent(self, agent: Agent, duration: float) -> bool:
        """Lands off the agent.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission land_agent.")
        
        ret_value = False

        agent.land()
        agent.set_trajectory_active(True)

        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= duration:
            t2_val = time.perf_counter()

        logging.info(f"Ending mission land_agent with success. Current height: {round(agent.get_pos()[2], 2)}")

        agent.set_trajectory_active(False)

        return ret_value

    def land_all(self, duration: float):
        """Lands all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        ret_value = False

        logging.info("Starting mission land_all.")

        for agent in self.__agents:
            ret_value = agent.land()
            agent.set_trajectory_active(True)
            agent.set_formation_active(False)
            agent.set_swarming(False)
        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= duration:
            self.__update()
            t2_val = time.perf_counter()


        logging.info(f"Ending missionLandAll with success. Total target count: {len(self.__agents)}")

        #agent.set_trajectory_active(False)

        return ret_value

    def goto_agent(self, target_agent: Agent, points: np.ndarray, duration: float) -> bool:
        """Moves the target agent to the specified point.

        Args:
            agent (Agent): Agent to be moved.
            points (np.ndarray): Points to be moved to.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        ret_value = False

        logging.info(f"Starting mission goto_agent. Target is '{target_agent.get_name()}'")

        # Itarete over the points
        for i, point in enumerate(points):
            target_agent.set_target_point(np.array([point[0], point[1], point[2]]))
            target_agent.set_trajectory_active(True)

            t1_val = time.perf_counter()
            t2_val = time.perf_counter()

            while t2_val - t1_val <= duration:
                self.__update()
                t2_val = time.perf_counter()

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goto_agent with success. Target was '{target_agent.get_name()}'")
                ret_value = True

            target_agent.set_trajectory_active(False)

        return ret_value

    def goto_swarm(self, points: np.ndarray, duration: float) -> bool:
        """Moves the swarm of agents to the specified point.

        Args:
            points (np.ndarray): Points to be moved to.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        ret_value = False

        logging.info(f"Starting mission goto_agent.")

        # Itarete over the points
        for i, point in enumerate(points):
            # Set target points and activate trajectory
            for agent in self.__agents:
                agent.set_target_point(np.array([point[0], point[1], point[2]]))
                agent.set_trajectory_active(True)

            t1_val = time.perf_counter()
            t2_val = time.perf_counter()

            while t2_val - t1_val <= duration:
                self.__update()
                t2_val = time.perf_counter()

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goto_swarm with success. Total target count: {len(self.__agents)}'")
                ret_value = True
            # Set target points and activate trajectory
            for agent in self.__agents:
                agent.set_trajectory_active(False)

        return ret_value

    def rotate_swarm(self, angle: float, duration: float):
        """Rotates the swarm.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        ret_value = False

        logging.info(f"Starting mission rotate_swarm.")

        for agent in self.__agents:
            agent.set_rotation(angle)
            agent.set_trajectory_active(False)
            agent.set_formation_const(0.4)
            agent.set_formation_active(True)

        t1_val = time.perf_counter()
        t2_val = time.perf_counter()

        while t2_val - t1_val <= duration:
            t2_val = time.perf_counter()

        for agent in self.__agents:
            agent.set_formation_const(0.9)

        logging.info(f"Ending rotate_swarm with success. Total target count: {len(self.__agents)}")

        ret_value = True

        return ret_value
    def kill_switch(self) -> bool:
        """Kills the agents in case of an emergency.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        ret_value = False

        for agent in self.__agents:
            agent.kill()
        ret_value = True
        return ret_value
    