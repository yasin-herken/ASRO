import logging
import redis
import numpy as np
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
                'name':agent.getName(),
                'adress':agent.getAddress(),
                'status':agent.getStatus(),
                'state':"HOVERING",
                'pos': agent.getPos(),
                'vel':agent.getVel(),
                'speed':agent.getSpeed(),
                'pitch':agent.getPitch(),
                'yaw':agent.getYaw(),
                'row':agent.getRoll()
            }
            self.data=json.dumps(self.sndMessage, cls=NumpyEncoder)
            self.__redisClient.set("channel", self.data)
        return retValue

    def goTo(self, target, maxSpeed) -> bool:
        """Made for testing purposes only. You can control the agents here like taking off, landing or etc.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        
        # TODO: Buraya istedigin kodu yazabilirsin. Dikkat et __redisClient suanlik kurulu degil.
        # TODO: __redisClient'in Redis'e baglanmasi lazim.
        
        # Simdilik takeOffAsync() yapiyor bu fonksiyon sadece.
        # timeHelper.sleep(0.01) olmazsa simulasyon nedense calismiyor ben de bilmiyorum.
        
        logging.info("Starting mission goTo.")

        for agent in self.__agents:
            agent.setTargetPoint(target)
            agent.setMaxVel(maxSpeed)
        
        while True:
            for agent in self.__agents:
                agent.update(self.__agents)
                
                if (not agent.isMoving()):
                    logging.info("Ending mission goTo with success.")
                    return True
            self.__crazySwarm.timeHelper.sleep(1 / 100)
                        
        return retValue

    def missionOne(self) -> bool:
        """_summary_

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        self.takeOffAll()

        self.__crazySwarm.timeHelper.sleep(5.0)

        self.goTo(np.array([1.0, -1.0, 1.1]), 0.7)
        self.goTo(np.array([-1.0, -1.0, 1.1]), 0.7)
        self.goTo(np.array([-1.0, 1.0, 1.1]), 0.7)
        self.goTo(np.array([1.0, 1.0, 1.1]), 0.7)
        self.goTo(np.array([0.0, 0.0, 1.1]), 0.7)

        self.landAll()

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

        logging.info(f"Starting mission takeOffAgent. Target is '{target}'")

        for agent in self.__agents:
            if target == agent.getName():
                agent.takeOffSync(1.0)
                self.__crazySwarm.timeHelper.sleep(5.0)
                retValue = True

        logging.info(f"Ending mission takeOffAgent with success. Target was '{target}'")

        return retValue
    
    def takeOffAll(self) -> bool:
        
        logging.info("Starting mission takeOffAll.")
        
        retValue = False
        for agent in self.__agents:
            agent.takeOffSync(1.0)
        self.__crazySwarm.timeHelper.sleep(5.0)
        retValue = True

        logging.info("Ending mission takeOffAll withsuccess")

        return retValue
    
    def landAll(self):
        retValue = False

        logging.info("Starting mission landAll.")
        
        for agent in self.__agents:
            agent.landAsync()
        self.__crazySwarm.timeHelper.sleep(5.0)
        retValue = True

        logging.info("Ending missionLandAll with success.")

        return retValue

    def landAgent(self, target: str) -> bool:
        """Lands the target agent.

        Args:
            target (str): Agent to be taken off.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission landAgent. Target is '{target}'")

        try:
           for agent in self.__agents:
            if target == agent.getName():
                #agent.landSync()
                pos = agent.getPos()
                target = np.array([pos[0], pos[1], 0.0])
                self.goTo(target, 0.2)
                retValue = True
        except Exception as e:
            print(e.with_traceback())
            
        logging.info(f"Ending mission landAgent with success. Target was '{target}'")

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
    