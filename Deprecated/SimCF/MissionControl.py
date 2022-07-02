import logging
import redis
import numpy as np
import json
import rospy
import Settings

from typing import List
from Agent import Agent


class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """
    def default(self, obj):
        if isinstance(obj, (np.int_, np.intc, np.intp, np.int8,
                            np.int16, np.int32, np.int64, np.uint8,
                            np.uint16, np.uint32, np.uint64)):
            return int(obj)
        elif isinstance(obj, (np.float_, np.float16, np.float32,
                              np.float64)):
            return float(obj)
        elif isinstance(obj, (np.ndarray,)):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class MissionControl:
    """Handles the agent operations depending on the mission on hand.
    """
    __agents: List[Agent]
    
    def __init__(self, agents: List[Agent]):
        """Initialize the MissionControl.

        Args:
            agents (List[Agent]): Agents to be operated.
        """
        self.__agents = agents

        return

    def testFormation(self) -> bool:
        """Takes the Agents into the specified formation.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        logging.info("Starting mission takeFormation.")

        # Check if formationMatrix matches the agent count
        formationMatrix = Settings.FORMATION_HEXAGON
        rowsCount = len(formationMatrix)
        isValid = True
        
        for i in range(rowsCount):
            columnsCount = len(formationMatrix[i])
            if rowsCount != columnsCount:
                isValid = False
        
        if (not isValid):
            logging.info("Formatin matrix is invalid. Rows and columns count do not match. Aborthing!")
            return False

        if (len(self.__agents) != rowsCount):
            logging.info(f"Agent count and formationMatrix does not match. Agents: {len(self.__agents)} formationMatrix: {rowsCount}x{rowsCount}")
            return False

        # Activate and give the formation parameters
        for agent in self.__agents:
            agent.setFormationMatrix(formationMatrix)
            agent.setFormationActive(True)

        # Wait for the formation to happen
        stoppedAgents = set()
        while True:
            for agent in self.__agents:

                if False:
                    stoppedAgents.add(agent.getName)
            
            if len(stoppedAgents) == len(self.__agents):
                retValue = True
                break

        logging.info(f"Ending mission takeFormation with success. Formation was: 'PYRAMID'")

        return retValue

    def missionOne(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        return retValue

    def missionTwo(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        return retValue

    def missionThree(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        return retValue

    def missionFour(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        return retValue

    def missionFive(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        return retValue

    def takeOffAgent(self, agent: Agent) -> bool:
        """Takes off the target agent.

        Args:
            agent (Agent): Agent to be taken off.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission takeOffAgent. Target is '{agent.getName()}'")
        
        currPos = agent.getPos()
        agent.setTargetPoint(np.array([0.0, 0.0, 0.5]))
        agent.setTargetHeight(0.5)
        rospy.sleep(1)

        while True:
            if (agent.getState() == "HOVERING"):
                retValue = True
                break

        logging.info(f"Ending mission takeOffAgent with success. Target was '{agent.getName()}'")

        return retValue
    
    def takeOffAll(self) -> bool:
        """Takes off all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission takeOffAll.")
        
        retValue = False

        for agent in self.__agents:
            retValue = self.takeOffAgent(agent)

        logging.info(f"Ending mission takeOffAll with success. Total target count: {len(self.__agents)}")

        return retValue
    
    def landAgent(self, agent: Agent) -> bool:
        """Lands the target agent.

        Args:
            agent (Agent): Agent to be landing.
        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission landAgent. Target is '{agent.getName()}'")

        currPos = agent.getPos()
        agent.setTargetPoint(np.array([currPos[0], currPos[1], 0.05]))
        agent.setTargetHeight(0.0)

        while True:
            if (agent.getState() == "STATIONARY"):
                retValue = True
                break
            
        logging.info(f"Ending mission landAgent with success. Target was '{agent.getName()}'")

        return retValue

    def landAll(self):
        """Lands all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info("Starting mission landAll.")
        
        for agent in self.__agents:
            retValue = self.landAgent(agent)

        logging.info(f"Ending missionLandAll with success. Total target count: {len(self.__agents)}")

        return retValue

    def goToAgent(self, agent: Agent, points: np.ndarray) -> bool:
        """Moves the target agent to the specified point.

        Args:
            agent (Agent): Agent to be moved.
            point (np.ndarray): Point to be moved to.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission goToAgent. Target is '{agent.getName()}'")

        # Itarete over the points
        for i, point in enumerate(points):
            agent.setTargetPoint(np.array([point[0], point[1], point[2]]))
            rospy.sleep(1)

            while True:
                if (agent.getState() == "HOVERING"):
                    break
            rospy.sleep(2)

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goToAgent with success. Target was '{agent.getName()}'")
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
    