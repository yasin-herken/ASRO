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

    def takeFormation(self, formationMatrix: np.ndarray) -> bool:
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
            agent.setTrajectoryActive(False)
            agent.setSwarming(True)

        # Wait for the formation to happen
        stoppedAgents = set()
        while True:
            self.__update()

            for agent in self.__agents:
                if  (
                        (agent.getName() not in stoppedAgents) and
                        agent.isInFormation()
                    ):
                    logging.info(f"Agent {agent.getName()} is in formation")
                    stoppedAgents.add(agent.getName())
                
            if len(stoppedAgents) == len(self.__agents):
                retValue = True
                break

        # Print the distances between
        for agent1 in self.__agents:
            for agent2 in self.__agents:
                if agent1 is not agent2:
                    print(f"{agent1.getName()} {agent2.getName()} -> {round(Settings.getDistance(agent1.getPos(), agent2.getPos()), 2)}")

        logging.info(f"Ending mission takeFormation with success. Formation was: 'HEXAGON'")

        return retValue

    def missionOne(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        return retValue

    def takeOffAgent(self, agent: Agent) -> bool:
        """Takes off the agent.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission takeOffAgent.")
        
        retValue = False

        agent.takeOff(0.5)

        while True:
            self.__update()

            if agent.getState() == "HOVERING":
                retValue = True
                break

        logging.info(f"Ending mission takeOffAgent with success. Current height: {round(agent.getPos()[2], 2)}")

        return retValue
    
    def takeOffAll(self) -> bool:
        """Takes off all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission takeOffAll.")
        
        retValue = False

        for agent in self.__agents:
            retValue = agent.takeOff(0.5)

        stoppedAgents = set()
        while True:
            self.__update()

            for agent in self.__agents:
                if agent.getState() == "HOVERING":
                    stoppedAgents.add(agent.getName())
            
            if (len(stoppedAgents) == len(self.__agents)):
                break

        logging.info(f"Ending mission takeOffAll with success. Total target count: {len(self.__agents)}")

        return retValue

    def landAgent(self, agent: Agent) -> bool:
        """Lands off the agent.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission landAgent.")
        
        retValue = False

        agent.land()

        while True:
            self.__update()

            if agent.getState() == "STATIONARY":
                retValue = True
                break

        logging.info(f"Ending mission landAgent with success. Current height: {round(agent.getPos()[2], 2)}")

        return retValue

    def landAll(self):
        """Lands all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info("Starting mission landAll.")

        for agent in self.__agents:
            agent.setTrajectoryActive(True)
            agent.setSwarming(False)
            retValue = agent.land()
        
        stoppedAgents = set()
        while True:
            self.__update()

            for agent in self.__agents:
                if (agent.getState() == "STATIONARY"):
                    stoppedAgents.add(agent.getName())
            
            if (len(stoppedAgents) == len(self.__agents)):
                break

        logging.info(f"Ending missionLandAll with success. Total target count: {len(self.__agents)}")

        return retValue

    def goToAgent(self, targetAgent: Agent, points: np.ndarray) -> bool:
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

            while True:
                self.__update()

                if (targetAgent.getState() == "HOVERING"):
                    break                

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goToAgent with success. Target was '{targetAgent.getName()}'")
                retValue = True
        
        return retValue

    def goToSwarm(self, points: np.ndarray) -> bool:
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
            self.__crazyServer.timeHelper.sleep(1)

            stoppedAgents = set()
            while True:
                self.__update()

                for agent in self.__agents:
                    if (agent.getState() == "HOVERING"):
                        stoppedAgents.add(agent.getName())

                if (len(stoppedAgents) == len(self.__agents)):
                    break               

            self.__crazyServer.timeHelper.sleep(2)

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goToSwarm with success. Total target count: {len(self.__agents)}'")
                retValue = True
        
        # Set target points and activate trajectory
        for agent in self.__agents:
            agent.setTrajectoryActive(False)
        
        return retValue

    def rotateSwarm(self, angle: float):
        """Rotates the swarm.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission rotateSwarm.")

        for agent in self.__agents:
            agent.setRotation(angle)

        stoppedAgents = set()

        while True:
            self.__update()

            for agent in self.__agents:
                if (agent.isInFormation()):
                    stoppedAgents.add(agent.getName())
            
            if (len(stoppedAgents) == len(self.__agents)):
                break

        logging.info(f"Ending rotateSwarm with success. Total target count: {len(self.__agents)}")

        retValue = True
        
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
    