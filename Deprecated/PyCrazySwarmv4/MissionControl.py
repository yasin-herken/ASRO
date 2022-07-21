import logging
import numpy as np
import Settings
import time

from typing import List
from Agent import Agent
from pycrazyswarm import Crazyswarm

class MissionControl:
    """Handles the agent operations depending on the mission on hand.
    """
    __agents: List[Agent]
    __crazyServer: Crazyswarm

    counter1 = time.perf_counter()
    counter2 = time.perf_counter()
    
    def __init__(self, agents: List[Agent], crazyServer: Crazyswarm):
        """Initialize the MissionControl.

        Args:
            agents (List[Agent]): Agents to be operated.
        """
        self.__agents = agents
        self.__crazyServer = crazyServer

        return

    def __update(self) -> bool:
        """Update the agents

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        
        self.__crazyServer.timeHelper.sleep(1 / 100)

        return True


    def __validateFormationMatrix(self, formationMatrix: np.ndarray) -> bool:
        rowsCount = len(formationMatrix)
        isValid = True
        retValue = True
        
        for i in range(rowsCount):
            columnsCount = len(formationMatrix[i])
            if rowsCount != columnsCount:
                isValid = False
        
        if (not isValid):
            logging.info("Formatin matrix is invalid. Rows and columns count do not match")
            retValue = False

        if (len(self.__agents) != rowsCount):
            logging.info(f"Agent count and formationMatrix does not match. Agents: {len(self.__agents)}, formationMatrix: {rowsCount}x{rowsCount}")
            retValue = False

        return retValue

    def takeFormation(self, formationMatrix: np.ndarray, duration: float) -> bool:
        """Takes the Agents into the specified formation.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        logging.info("Starting mission takeFormation.")

        # Check if formationMatrix matches the agent count
        if (not self.__validateFormationMatrix(formationMatrix)):
            logging.info("Aborting!")
            return False

        # Activate and give the formation parameters
        for agent in self.__agents:
            agent.setFormationMatrix(formationMatrix)
            agent.setFormationActive(True)
            agent.setSwarming(True)

        # Wait for the formation to happen
        t1 = time.perf_counter()
        t2 = time.perf_counter()

        while t2 - t1 <= duration:
            self.__update()
            t2 = time.perf_counter()

        # Print the distances between
        for agent1 in self.__agents:
            for agent2 in self.__agents:
                if agent1 is not agent2:
                    print(f"{agent1.getName()} {agent2.getName()} -> {round(Settings.getDistance(agent1.getPos(), agent2.getPos()), 2)}")

        logging.info(f"Ending mission takeFormation with success.")

        return retValue

    def missionOne(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        return retValue

    def takeOffAgent(self, agent: Agent, height: float, duration: float) -> bool:
        """Takes off the agent.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission takeOffAgent.")
        
        retValue = False

        agent.takeOff(height)
        agent.setTrajectoryActive(True)

        t1 = time.perf_counter()
        t2 = time.perf_counter()

        while t2 - t1 <= duration:
            self.__update()
            t2 = time.perf_counter()


        logging.info(f"Ending mission takeOffAgent with success. Current height: {round(agent.getPos()[2], 2)}")

        agent.setTrajectoryActive(False)

        return retValue
    
    def takeOffAll(self, height: float, duration: float) -> bool:
        """Takes off all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission takeOffAll.")
        
        retValue = False

        for agent in self.__agents:
            retValue = agent.takeOff(height)
            agent.setTrajectoryActive(True)
    
        t1 = time.perf_counter()
        t2 = time.perf_counter()

        while t2 - t1 <= duration:
            self.__update()
            t2 = time.perf_counter()


        logging.info(f"Ending mission takeOffAll with success. Total target count: {len(self.__agents)}")

        agent.setTrajectoryActive(False)

        return retValue

    def landAgent(self, agent: Agent, duration: float) -> bool:
        """Lands off the agent.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission landAgent.")
        
        retValue = False

        agent.land()
        agent.setTrajectoryActive(True)

        t1 = time.perf_counter()
        t2 = time.perf_counter()

        while t2 - t1 <= duration:
            self.__update()
            t2 = time.perf_counter()

        logging.info(f"Ending mission landAgent with success. Current height: {round(agent.getPos()[2], 2)}")

        agent.setTrajectoryActive(False)

        return retValue

    def landAll(self, duration: float):
        """Lands all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info("Starting mission landAll.")

        for agent in self.__agents:
            retValue = agent.land()
            agent.setTrajectoryActive(True)
            agent.setSwarming(False)
        
        t1 = time.perf_counter()
        t2 = time.perf_counter()

        while t2 - t1 <= duration:
            self.__update()
            t2 = time.perf_counter()


        logging.info(f"Ending missionLandAll with success. Total target count: {len(self.__agents)}")

        agent.setTrajectoryActive(False)

        return retValue

    def goToAgent(self, targetAgent: Agent, points: np.ndarray, duration: float) -> bool:
        """Moves the target agent to the specified point.

        Args:
            agent (Agent): Agent to be moved.
            points (np.ndarray): Points to be moved to.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission goToAgent. Target is '{targetAgent.getName()}'")

        # Itarete over the points
        for i, point in enumerate(points):
            targetAgent.setTargetPoint(np.array([point[0], point[1], point[2]]))
            targetAgent.setTrajectoryActive(True)

            t1 = time.perf_counter()
            t2 = time.perf_counter()

            while t2 - t1 <= duration:
                self.__update()
                t2 = time.perf_counter()

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goToAgent with success. Target was '{targetAgent.getName()}'")
                retValue = True

            targetAgent.setTrajectoryActive(False)
        
        return retValue

    def goToSwarm(self, points: np.ndarray, duration: float) -> bool:
        """Moves the swarm of agents to the specified point.

        Args:
            points (np.ndarray): Points to be moved to.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission goToAgent.")

        # Itarete over the points
        for i, point in enumerate(points):
            # Set target points and activate trajectory
            for agent in self.__agents:
                agent.setTargetPoint(np.array([point[0], point[1], point[2]]))
                agent.setTrajectoryActive(True)

            t1 = time.perf_counter()
            t2 = time.perf_counter()

            while t2 - t1 <= duration:
                self.__update()
                t2 = time.perf_counter()

            self.__crazyServer.timeHelper.sleep(2)

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goToSwarm with success. Total target count: {len(self.__agents)}'")
                retValue = True
        
            # Set target points and activate trajectory
            for agent in self.__agents:
                agent.setTrajectoryActive(False)
        
        return retValue

    def rotateSwarm(self, angle: float, duration: float):
        """Rotates the swarm.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission rotateSwarm.")

        for agent in self.__agents:
            agent.setRotation(angle)
            agent.setFormationActive(True)

        t1 = time.perf_counter()
        t2 = time.perf_counter()

        while t2 - t1 <= duration:
            self.__update()
            t2 = time.perf_counter()

        logging.info(f"Ending rotateSwarm with success. Total target count: {len(self.__agents)}")

        retValue = True
        
        for agent in self.__agents:
            agent.setFormationActive(False)

        return retValue
    
    def killSwitch(self) -> bool:
        """Kills the agents in case of an emergency.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        retValue = False

        for agent in self.__agents:
            agent.kill()
        retValue = True
        
        return retValue
    