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

class Point:
    x: float
    y: float
    z: float

    arrived: bool

    def __init__(self, x, y, z, arrived):
        self.x = x
        self.y = y
        self.z = z
        self.arrived = arrived

class MissionControl:
    """Handles the agent operations depending on the mission on hand.
    """
    
    __rospyRate: rospy.Rate
    __agents: List[Agent]
    __redisClient: redis.Redis
    
    def __init__(self, rospyRate: rospy.Rate, agents: List[Agent],redisClient:redis.Redis):
        """Initialize the MissionControl.

        Args:
            crazySwarm (Crazyswarm): Crazyswarm server (ROS and other stuff).
            agents (List[Agent]): Agents to be operated.
        """
        self.__rospyRate = rospyRate
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
                        
        return retValue

    def testFormation(self) -> bool:
        """Takes the Agents into the specified formation.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False
        logging.info("Starting mission takeFormation.")

        # Activate and give the formation parameters
        for agent in self.__agents:
            agent.setFormationControl(True)
            agent.setFormationMatrix(Settings.FORMATION_PYRAMID)

        # Wait for the formation to happen
        stoppedAgents = set()
        while True:

            for agent in self.__agents:
                agent.update(self.__agents)
                if not agent.isMoving():
                    stoppedAgents.add(agent.getName)
            
            if len(stoppedAgents) == len(self.__agents):
                retValue = True
                break

            self.__crazySwarm.timeHelper.sleep(1 / 100)

        # Deactivate the formation parameters
        for agent in self.__agents:
            agent.setFormationControl(False)
            agent.setFormationMatrix(np.array([0.0]))

        logging.info(f"Ending mission takeFormation with success. Formation was: 'PYRAMID'")

        return retValue
    
    def testPosition(self) -> bool:
        """__summary__

        Returns:
            bool: Specifies whether the operation was successfull or not.
        """
        retValue = False

        while True:
            for agent in self.__agents:
                agent.update(self.__agents, isActive=False)
                pos = agent.getPos()
                print(f"{agent.getName()} x: {round(pos[0], 2)}, y: {round(pos[1], 2)}, z: {round(pos[2], 2)}")
                
            self.__rospyRate.sleep()

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
    
    def hoverAgent(self, target: str) -> bool:
        """Hovers the target agent at it's current position.

        Args:
            target (str): Agent to be hovered.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission takeOffAgent. Target is '{target}'")

        for agent in self.__agents:
            if target == agent.getName():
                target = agent
                break

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
                target = agent
                break
        
        agent.update(self.__agents)
        currPos = agent.getPos()
        agent.setTargetPoint(np.array([currPos[0], currPos[1], currPos[2] + 0.75]))
        agent.setMaxVel(1.0)

        while True:
            agent.update(self.__agents)

            if (not agent.isMoving()):
                retValue = True
                break

            self.__rospyRate.sleep()

        logging.info(f"Ending mission takeOffAgent with success. Target was '{target.getName()}'")

        return retValue
    
    def takeOffAll(self) -> bool:
        """Takes off all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        logging.info("Starting mission takeOffAll.")
        
        retValue = False

        for agent in self.__agents:
            retValue = self.takeOffAgent(agent.getName())

        logging.info(f"Ending mission takeOffAll with success. Total target count: {len(self.__agents)}")

        return retValue
    
    def landAll(self):
        """Lands all the agents.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info("Starting mission landAll.")
        
        for agent in self.__agents:
            retValue = self.landAgent(agent.getName())

        logging.info(f"Ending missionLandAll with success. Total target count: {len(self.__agents)})")

        return retValue

    def landAgent(self, target: str) -> bool:
        """Lands the target agent.

        Args:
            target (str): Agent to be landing.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission landAgent. Target is '{target}'")

        for agent in self.__agents:
            if target == agent.getName():
                target = agent
                break
        
        agent.update(self.__agents)
        currPos = agent.getPos()
        agent.setTargetPoint(np.array([currPos[0], currPos[1], 0.1]))
        agent.setMaxVel(0.1)

        while True:
            agent.update(self.__agents)

            if (agent.getState() == "LANDED"):
                retValue = True
                break

            self.__rospyRate.sleep()

            
        logging.info(f"Ending mission landAgent with success. Target was '{target.getName()}'")

        return retValue

    def goToAgent(self, target: str, points: List[Point], maxVel: float) -> bool:
        """Moves the target agent to the specified point.

        Args:
            point (np.ndarray): Point to be moved to.
            target (str): Agent to be moved.

        Returns:
            bool: Specifies whether the mission was successfull or not.
        """
        retValue = False

        logging.info(f"Starting mission goToAgent. Target is '{target}'")

        # Find the target Agent object
        for agent in self.__agents:
            if agent.getName == target:
                target = agent
                break

        # Itarete over the points
        for i, point in enumerate(points):
            agent.setTargetPoint(np.array([point.x, point.y, point.z]))
            agent.setMaxVel(maxVel)

            while True:
                agent.update(self.__agents)

                if (not agent.isMoving()):
                    point.arrived = True
                    break

                self.__rospyRate.sleep()

            # Last point
            if i == len(points) - 1:
                logging.info(f"Ending mission goToAgent with success. Target was '{target}'")
                retValue = True
        
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
    