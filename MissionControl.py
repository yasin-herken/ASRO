import redis
import numpy as np
import marshal
import json
from typing import List
from Agent import Agent
from pycrazyswarm import Crazyswarm
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
    
    __crazySwarm: Crazyswarm
    __agents: List[Agent]
    __redisClient: redis.Redis
    
    def __init__(self, crazySwarm: Crazyswarm, agents: List[Agent],redisClient:redis.Redis):
        """Initialize the MissionControl.

        Args:
            crazySwarm (Crazyswarm): Crazyswarm server (ROS and other stuff).
            agents (List[Agent]): Agents to be operated.
        """
        self.__crazySwarm = crazySwarm
        self.__agents = agents
        self.__redisClient=redisClient
        self.__redisClient=redis.Redis(host="localhost",port=6379)
        return
    def _syncRedis(self) -> bool:
        """Sends to agents' info to the Redis server as 'cf{no}_{param}'.

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        
        retValue = True
        for agent in self.__agents:
            self.sndMessage={
                '__name':agent.getName(),
                '__adress':agent.getAddress(),
                '__status':'online',
                '__state':agent.getStatus(),
                '__pos': agent.getPos(),
                '__vel':agent.getVel(),
                '__speed':agent.getSpeed(),
                '__pitch':agent.getPitch(),
                '__yaw':agent.getYaw(),
                '__row':agent.getRow()
            }
            self.data=json.dumps(self.sndMessage, cls=NumpyEncoder)
            self.__redisClient.set("channel",self.data)
        return retValue

    def missionZero(self) -> bool:
        """Made for testing purposes only. You can control the agents here like taking off, landing or etc.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        # TODO: Buraya istedigin kodu yazabilirsin. Dikkat et __redisClient suanlik kurulu degil.
        # TODO: __redisClient'in Redis'e baglanmasi lazim.
        
        # Simdilik takeOffAsync() yapiyor bu fonksiyon sadece.
        # timeHelper.sleep(0.01) olmazsa simulasyon nedense calismiyor ben de bilmiyorum.
        
        self.__agents[0].takeOffAsync(1.5)
        
        while True:
            self.__crazySwarm.timeHelper.sleep(0.1)
            self._syncRedis()
            rcv_data=self.__redisClient.get("channel")
            message=json.loads(rcv_data)
            print(type(message))
            print(message)
        
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
    