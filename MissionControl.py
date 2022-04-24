import redis

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
    
    def __syncRedis(self):
        pass

    def missionOne(self):
        pass

    def missionTwo(self):
        pass

    def missionThree(self):
        pass

    def missionFour(self):
        pass

    def missionFive(self):
        pass

    def takeOffAgent(self, target: Agent):
        pass

    def landAgent(self, target: Agent):
        pass

    def goToAgent(self, target: Agent):
        pass
    
    def killSwitch(self, isOn: bool):
        pass
    