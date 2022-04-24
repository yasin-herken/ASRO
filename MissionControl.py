import redis
import numpy as np
from Agent import Agent
from pycrazyswarm import Crazyswarm

class MissionControl:
    """Handles the agent operations depending on the mission on hand.
    """
    
    __agents: list[Agent]
    __redisClient: redis.Redis
    
    def __init__(self, agents: list[Agent]):
        """Initialize the MissionControl.

        Args:
            agents (list[Agent]): Agents to be operated.
        """
        pass
    
    def __syncRedis(self) -> bool:
        """Sends to agents' info to the Redis server as 'cf{no}_{param}'.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        retValue = False
        
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

    def takeOffAgent(self, target: str) -> bool:
        """Takes off the target agent.

        Args:
            target (str): Agent to be taken off.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        return retValue

    def landAgent(self, target: str) -> bool:
        """Lands the target agent.

        Args:
            target (str): Agent to be taken off.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        return retValue

    def goToAgent(self, point: np.ndarray, target: str) -> bool:
        """Moves the target agent to the specified point.

        Args:
            point (np.ndarray): Point to be moved to.
            target (str): Agent to be moved.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        return retValue
    
    def killSwitch(self, isOn: bool) -> bool:
        """Kills the agents in case of an emergency.

        Args:
            isOn (bool): Safety value. The function only activates if this value is true.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        retValue = False
        
        return retValue
    