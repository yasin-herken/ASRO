import logging
from turtle import speed
import numpy as np
import Settings
import time
import rospy

from sim_cf.crazyflie import Crazyflie

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import FullState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class Agent:
    """This class represents the real-world agent.
    It does swarming and other operations to control the agent.
    """
    __crazyflie: Crazyflie
    __name: str
    __state: str
    
    __initialPos: np.ndarray
    __pos: np.ndarray
    __vel: np.ndarray
    __speed: float

    # Swarm related
    __isFormationActive: bool
    __isAvoidanceActive: bool
    __isTrajectoryActive: bool
    __isSwarming: bool
    __formationMatrix: np.ndarray

    __targetPoint: np.ndarray
    __targetHeight: float
    
    # time points
    __t1: float
    __t2: float
    
    # position points
    __x1: np.ndarray
    __x2: np.ndarray

    # The publisher to be able to control in position
    __pubHover: rospy.Publisher
    __pubStop: rospy.Publisher
    __pubFullState: rospy.Publisher
    __pubTwist: rospy.Publisher

    # Message structures
    __hoverMsg: Hover
    __fullStateMsg: FullState
    
    def __init__(self, cf: Crazyflie, initialPos: np.ndarray, name: str) -> None:
        """Initializes the agent class.

        Args:
            cf (crazyflie): Crazyflie handler to command the realworld agent.
            name (str): Name of the agent.
            address (str): Address of the agent.
        """
        self.__crazyflie = cf
        self.__name = name
        
        self.__isFormationActive = False
        self.__isAvoidanceActive = False
        self.__isTrajectoryActive = True
        self.__isSwarming = False
        self.__formationMatrix = np.array([0.0])

        self.__targetPoint = np.array([0.0, 0.0, 0.0])
        self.__targetHeight = 0.5
        
        self.__state = "STATIONARY"
        
        self.__initialPos = np.array(initialPos)
        self.__pos = np.array(initialPos)
        self.__vel = np.array([0.0, 0.0, 0.0])
        self.__speed = 0.0
        self.__maxSpeed = 0.2

        self.__t1 = time.perf_counter()
        self.__t2 = time.perf_counter()
        
        self.__x1 = np.array([0.0, 0.0, 0.0])
        self.__x2 = np.array([0.0, 0.0, 0.0])
        
        self.__validtyCount = 0

        # Change the controller (1 is PID, 2 is Mellinger)
        self.__crazyflie.setParam("stabilizer/controller", 1) 

        # Subscribe to get the local position of the crazyflie with prefix cf_prefix
        rospy.Subscriber(self.__name + "/local_position" , GenericLogData , self.__localPositionCallback)
        rospy.Subscriber(self.__name + "/external_position" , GenericLogData , self.__externalPositionCallback)

        # The publisher to be able to control in position
        self.__pubHover = rospy.Publisher(self.__name + "/cmd_hover", Hover , queue_size=10)
        self.__pubStop = rospy.Publisher(self.__name + "/cmd_stop", Empty , queue_size=10)
        self.__pubFullState = rospy.Publisher(self.__name + "/cmd_full_state", FullState , queue_size=10)
        self.__pubTwist= rospy.Publisher(self.__name + "/cmd_vel", Twist , queue_size=10)
        rospy.sleep(0.1)

        # Message structures
        self.__hoverMsg = Hover()
        self.__fullStateMsg = FullState()

    def __localPositionCallback(self, msg):
        self.__pos[0] = msg.values[0]
        self.__pos[1] = msg.values[1]
        self.__pos[2] = msg.values[2]

        # Update agent info
        self.__x1 = np.array(self.__x2)
        self.__x2 = np.array(self.__pos)

        self.__t2 = time.perf_counter()
        # print(round(self.__t2, 6))
        self.__vel = (self.__x2 - self.__x1) / (self.__t2 - self.__t1)
        self.__t1 = time.perf_counter()
        
        self.__speed = Settings.getMagnitude(self.__vel)

    def __externalPositionCallback(self, msg):
        return
        self.__pos[0] = msg.values[0]
        self.__pos[1] = msg.values[1]
        self.__pos[2] = msg.values[2]

        # Update agent info
        self.__x1 = self.__x2
        self.__x2 = self.__pos
        
        self.__t2 = time.perf_counter()
        self.__vel = (self.__x2 - self.__x1) / (self.__t2 - self.__t1)
        self.__t1 = time.perf_counter()
        
        self.__speed = Settings.getMagnitude(self.__vel)  

    def __formationControl(self, agents: list) -> np.ndarray:
        """Calculates the formation 'force' to be applied to the agent.
        This calculation moves the agent into formation.
        
        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        retValue = np.array([0.0, 0.0, 0.0])

        # Find your own index
        idx = 0
        for i, agent in enumerate(agents):
            if agent == self:
                idx = i
                break

        # Itarete over agents and calculate the desired vectors
        for i, agent in enumerate(agents):
            if agent != self:
                distanceToOtherAgent = agent.getPos() - self.getPos()

                distanceToDesiredPoint = Settings.setMagnitude(
                    distanceToOtherAgent,
                    self.__formationMatrix[idx][i]
                )
            
                retValue += (distanceToOtherAgent - distanceToDesiredPoint)

        return retValue
    
    def __avoidanceControl(self, agents: list) -> np.ndarray:
        """Calculates the avoidance 'force' to be applied to the agent.
        This calculation keeps the agent away from the obstacles.

        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        pass
    
    def __trajectoryControl(self, agents: list) -> np.ndarray:
        """Calculates the trajectory 'force' to be applied to the agent.
        This calculation moves the agent towards the target point.

        Args:
            agents (list): List of agents

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        retValue = np.array([0.0, 0.0, 0.0])
        swarmCenter = np.array([0.0, 0.0, 0.0])

        if self.__isSwarming:
            for otherAgent in agents:
                swarmCenter += otherAgent.getPos()
            swarmCenter /= len(agents)
        else:
            swarmCenter = self.getPos()

        retValue = self.__targetPoint - swarmCenter
        
        return retValue
    
    def __estimateState(self, trajectoryVel) -> bool:
        retValue = "UNKOWN"

        height = self.__pos[2]
        desiredSpeed = Settings.getMagnitude(trajectoryVel)
        desiredVerticalSpeed = trajectoryVel[2]

        heightLimit = max(self.__targetHeight + 0.05, 0.55)
        speedLimit = self.__maxSpeed + 0.05
        
        toleranceVal = 0.05

        if (
                (0.00 <=  height <= 0.15) and
                (0.00 <= desiredSpeed <= toleranceVal) and
                (0.00 <= desiredVerticalSpeed <= toleranceVal)
            ):
            retValue = "STATIONARY"
        elif (
                (0.00 <=  height <= heightLimit) and
                (0.00 <= desiredSpeed <= toleranceVal) and
                (-toleranceVal <= desiredVerticalSpeed <= toleranceVal)
            ):
            retValue = "HOVERING"
        elif (
                (0.00 <=  height <= heightLimit) and
                (0.00 <= desiredSpeed <= speedLimit) and
                (-toleranceVal <= desiredVerticalSpeed <= toleranceVal)
            ):
            retValue = "MOVING"
        elif (
                (0.00 <=  height <= heightLimit) and
                (0.00 <= desiredSpeed <= speedLimit) and
                (0.00 <= desiredVerticalSpeed <= speedLimit)
            ):
            retValue = "TAKING_OFF"
        elif (
                (0.00 <= height <= heightLimit) and
                (0.00 <= desiredSpeed <= speedLimit) and
                (-speedLimit <= desiredVerticalSpeed <= -toleranceVal)
            ):
            retValue = "LANDING"
        else:
            logging.info(f"Unhandled state height: {round(height, 3)}, desiredSpeed: {round(desiredSpeed,3 )}, desiredVerticalSpeed: {round(desiredVerticalSpeed, 3)}")
            logging.info(f"Values in hand heightLimit: {round(heightLimit, 3)} speedLimit: {round(speedLimit, 3)} toleranceVal: {round(toleranceVal, 3)}")
            
        return retValue

    def getName(self) -> str:
        return self.__name
    
    def getState(self) -> str:
        return self.__state
    
    def getPos(self) -> np.ndarray:
        return self.__pos
    
    def getVel(self) -> np.ndarray:
        return self.__vel
    
    def getSpeed(self) -> float:
        return Settings.getMagnitude(self.__vel)
    
    def getInitialPos(self) -> np.ndarray:
        return self.__initialPos
    
    def setTargetPoint(self, target: np.ndarray) -> bool:
        self.__targetPoint = target
        logging.info(f"Target point set x: {round(self.__targetPoint[0], 2)} y: {round(self.__targetPoint[1], 2)} z: {round(self.__targetPoint[2], 2)}")
    
    def setTargetHeight(self, targetHeight: float) -> bool:
        self.__targetHeight = targetHeight

    def setMaxSpeed(self, maxSpeed: float) -> bool:
        self.__maxSpeed = maxSpeed

    def setFormationActive(self, status: bool) -> bool:
        self.__isFormationActive = status

    def setTrajectoryActive(self, status: bool) -> bool:
        self.__isTrajectoryActive = status
    
    def setFormationMatrix(self, matrix: np.ndarray) -> bool:
        self.__formationMatrix = matrix
    
    def setSwarming(self, swarming: bool) -> bool:
        self.__isSwarming = swarming

    def update(self, agents: list) -> bool:
        """Depending on the settings, calculates the control values and applies them. 

        Args:
            agents (list): List of all agents in the system.

        Returns:
            bool: Whether the update was succesfull or not.
		"""
        retValue = False

        # Calculate control values
        controlVel = np.array([0.0, 0.0, 0.0])
        formationVel = np.array([0.0, 0.0, 0.0])
        avoidanceVel = np.array([0.0, 0.0, 0.0])
        trajectoryVel = np.array([0.0, 0.0, 0.0])

        if self.__isFormationActive:
            formationVel = self.__formationControl(agents)

        if self.__isAvoidanceActive:
            avoidanceVel = self.__avoidanceControl()

        if self.__isTrajectoryActive:
            trajectoryVel = self.__trajectoryControl(agents)

        # Limit swarm control values
        if self.__maxSpeed < Settings.getMagnitude(trajectoryVel):
            trajectoryVel = Settings.setMagnitude(trajectoryVel, self.__maxSpeed)
        

        # Update the state of the agent
        newState = self.__estimateState(trajectoryVel)
        if self.__state != newState:
            logging.info(f"Changing state {self.__state} -> {newState}")
            self.__state = newState

        controlVel = formationVel + avoidanceVel + trajectoryVel

        # Prepare FullState Msg
        self.__fullStateMsg.header.frame_id = 'world'
        self.__fullStateMsg.header.seq += 1
        self.__fullStateMsg.header.stamp = rospy.Time.now()
        self.__fullStateMsg.pose.position.x = self.__pos[0] + controlVel[0]
        self.__fullStateMsg.pose.position.y = self.__pos[1] + controlVel[1]
        self.__fullStateMsg.pose.position.z = self.__targetHeight

        # Prepare Hover Msg
        self.__hoverMsg.header.frame_id = 'world'
        self.__hoverMsg.header.seq += 1
        self.__hoverMsg.header.stamp = rospy.Time.now()
        self.__hoverMsg.vx = controlVel[0]
        self.__hoverMsg.vy = controlVel[1]
        self.__hoverMsg.yawrate = 0.0
        self.__hoverMsg.zDistance = self.__targetHeight

        # Send the commanding message
        if self.__state == "STATIONARY":
            pass
        elif self.__state == "MOVING":
            self.__pubFullState.publish(self.__fullStateMsg)
        else:
            self.__pubHover.publish(self.__hoverMsg)

        return retValue

    def kill(self) -> bool:
        """Stops all the agent motors.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = True

        for i in range(10):
            self.__pubStop.publish(Empty())

        return retValue
    
    