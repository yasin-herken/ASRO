import logging
import numpy as np
import Settings
import time
import rospy

# Remove the word 'Sim' in order to run IRL !!!
from sim_cf.crazyflie import Crazyflie

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import FullState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

# TODO: Fix the private values "_" to "__"

class Agent:
    """This class represents the real-world agent.
    It does swarming and other operations to control the agent.
    """
    __name: str
    __address: str
    _status:  bool
    _state: str
    
    __initialPos: np.ndarray
    __pos: np.ndarray
    __vel: np.ndarray
    __speed: float
    
    __pitch: float
    __yaw: float
    __roll: float

    # Swarm related
    __isFormationActive: bool
    __isAvoidanceActive: bool
    __isTrajectoryActive: bool
    __isSwarming: bool
    __formationMatrix: np.ndarray

    _targetPoint: np.ndarray

    __crazyflie: Crazyflie
    
    # time points
    __t1: float
    __t2: float
    __maxVel: float 
    
    # position points
    __x1: np.ndarray
    __x2: np.ndarray
    
    # validity
    __validtyCount: int

    # The publisher to be able to control in position
    __pubSetpointPos: rospy.Publisher
    __pubHover: rospy.Publisher
    __pubStop: rospy.Publisher
    __pubFullState: rospy.Publisher
    __pubStop: rospy.Publisher
    __pubTwist: rospy.Publisher

    # Message structures
    __hoverMsg: Hover
    __fullStateMsg: FullState
    
    def __init__(self, cf: Crazyflie, initialPos: np.ndarray, name: str, address: str) -> None:
        """Initializes the agent class.

        Args:
            cf (crazyflie): Crazyflie handler to command the realworld agent.
            name (str): Name of the agent.
            address (str): Address of the agent.
        """
        self.__crazyflie = cf
        self.__name = name
        self.__address = address

        self.__maxVel = 0.3
        
        self.__isFormationActive = False
        self.__isAvoidanceActive = False
        self.__isTrajectoryActive = False
        self.__isSwarming = False
        self.__formationMatrix = np.array([0.0])

        self._targetPoint = np.array([0.0, 0.0, 0.0])
        
        self._status = False
        self._state = "STATIONARY"
        
        self.__initialPos = np.array(initialPos)
        self.__pos = np.array(initialPos)
        self.__vel= np.array([0.0, 0.0, 0.0])
        self.__speed = 0.0
        
        self.__pitch= 0.0
        self.__yaw= 0.0
        self.__roll = 0.0

        self.__t1 = time.perf_counter()
        self.__t2 = time.perf_counter()
        
        self.__x1 = np.array([0.0, 0.0, 0.0])
        self.__x2 = np.array([0.0, 0.0, 0.0])
        
        self.__validtyCount = 0

        # Subscribe to get the local position of the crazyflie with prefix cf_prefix
        rospy.Subscriber(self.__name + "/local_position" , GenericLogData , self.__localPositionCallback)
        rospy.Subscriber(self.__name + "/external_position" , GenericLogData , self.__externalPositionCallback)

        # The publisher to be able to control in position
        self.__pubSetPointPos = rospy.Publisher(self.__name + "/cmd_position", Position , queue_size=10)
        self.__pubHover = rospy.Publisher(self.__name + "/cmd_hover", Hover , queue_size=10)
        self.__pubStop = rospy.Publisher(self.__name + "/cmd_stop", Empty , queue_size=10)
        self.__pubFullState = rospy.Publisher(self.__name + "/cmd_full_state", FullState , queue_size=10)
        self.__pubStop = rospy.Publisher(self.__name + "/cmd_stop", Empty , queue_size=10)
        self.__pubTwist= rospy.Publisher(self.__name + "/cmd_vel", Twist , queue_size=10)
        rospy.sleep(1 / 10)

        # Message structures
        self.__hoverMsg = Hover()
        self.__fullStateMsg = FullState()
        self.__PointMsg = Position()

        # Activate service controls
        self.__crazyflie.setParam("commander/enHighLevel", 1)
        
    def __str__(self):
        return f"{self.__name}, {self.__address}, {self.__pos}"

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

        retValue = self._targetPoint - swarmCenter
        
        # max velocity 1.0
        if self.__maxVel < Settings.getMagnitude(retValue):
            retValue = Settings.setMagnitude(retValue, self.__maxVel)
        
        return retValue

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

        print("external")

        # Update agent info
        self.__x1 = self.__x2
        self.__x2 = self.__pos
        
        self.__t2 = time.perf_counter()
        self.__vel = (self.__x2 - self.__x1) / (self.__t2 - self.__t1)
        self.__t1 = time.perf_counter()
        
        self.__speed = Settings.getMagnitude(self.__vel)  

    def getName(self) -> str:
        """Returns the name of the agent.

        Returns:
            str: Name of the agent.
        """
        return self.__name
    
    def getAddress(self) -> str:
        """Returns the address of the agent.

        Returns:
            str: Address of the agent.
        """
        return self.__address
    
    def getStatus(self) -> bool:
        """Returns status name of the agent.

        Returns:
            bool: Status of the agent.
        """

        return "online"
    
    def getState(self) -> str:
        """_summary_

        Returns:
            str: State of the agent
        """
        
        return self._state
    
    def getPos(self) -> np.ndarray:
        """Returns the position of the agent. (Vector3)

        Returns:
            np.ndarray: Position of the agent.
        """
        return self.__pos
    
    def getVel(self) -> np.ndarray:
        """Returns the velocity of the agent. (Vector3)

        Returns:
            np.ndarray: Velocity of the agent.
        """
        
        return self.__vel
    
    def getSpeed(self) -> float:
        """Returns the speed of the agent. (Scaler)

        Returns:
            float: Speed of the agent.
        """
        
        return Settings.getMagnitude(self.__vel)
    
    def getPitch(self) -> float:
        """Returns the pitch of the agent.

        Returns:
            float: Pitch of the agent.
        """
        return self.__pitch
    
    def getYaw(self) -> float:
        """Returns the yaw of the agent.

        Returns:
            float: Yaw of the agent.
        """
        (self.__roll, self.__pitch, self.__yaw) = self.__crazyflie.rpy()
        return self.__yaw
    
    def getRoll(self) -> float:
        """Returns the row of the agent.

        Returns:
            float: Row of the agent.
        """
        (self.__roll, self.__pitch, self.__yaw) = self.__crazyflie.rpy()
        return self.__roll
    
    def getInitialPos(self) -> np.ndarray:
        """Returns the initial position of the agent.

        Returns:
            np.ndarray: Initial position of the agent.
        """
        return self.__initialPos
    
    def setTargetPoint(self, target) -> bool:
        self._targetPoint = target
        self._state = "MOVING"
        
        return True
    
    def setMaxVel(self, vel) -> bool:
        self.__maxVel = vel

        return True

    def setFormationActive(self, status: bool) -> bool:
        self.__isFormationActive = status

        return True

    def setTrajectoryActive(self, status: bool) -> bool:
        self.__isTrajectoryActive = status

        return True
    
    def setFormationMatrix(self, matrix: np.ndarray) -> bool:
        self.__formationMatrix = matrix

        return True
    
    def setSwarming(self, swarming: bool) -> bool:
        self.__isSwarming = swarming

        return True
    
    def update(self, agents: list, isActive = True) -> bool:
        """Retrieves the agent information and updates the member veriables.
        Depending on the settings, calculates the formation control values and applies them. 

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
            formationVel = Settings.setMagnitude(formationVel, 0.5)

        if self.__isAvoidanceActive:
            avoidanceVel = self.__avoidanceControl()

        if self.__isTrajectoryActive:
            trajectoryVel = self.__trajectoryControl(agents)

        controlVel = formationVel + avoidanceVel + trajectoryVel
        if 0.06 <= Settings.getMagnitude(controlVel):
            controlVel = Settings.setMagnitude(controlVel, 0.06)

        isHighSpeed = False if (self.__speed <= 0.05) else True
        isHighControl = False if (Settings.getMagnitude(controlVel) <= 0.05) else True

        # Calculate is states
        if self._state == "TAKING_OFF" and 0.49 <= self.__pos[2]:
            self._state = "HOVERING"
            if self.__validtyCount < 0:
                self.__validtyCount -= 1
        elif self._state == "LANDING" and self.__pos[2] <= 0.15 and self._targetPoint[2] <= 0.10:
            self._state = "STATIONARY"
            if self.__validtyCount < 0:
                self.__validtyCount -= 1
        elif self._state == "STATIONARY":
            pass
        elif self._state != "TAKING_OFF" and not isHighSpeed and not isHighControl:
            self.__validtyCount += 1
            if 180 <= self.__validtyCount:
                self._state = "HOVERING"
                self.__validtyCount = 0
        elif isHighSpeed and isHighControl:
            self._state = "MOVING"
            if self.__validtyCount < 0:
                self.__validtyCount -= 1
        
        print(self._state)
        # print(round(self.__speed, 4))

        # Send the message
        if isActive:
            # Hover Messages
            self.__hoverMsg.header.frame_id = 'world'
            self.__hoverMsg.header.seq += 1
            self.__hoverMsg.header.stamp = rospy.Time.now()
            self.__hoverMsg.vx = 0.0
            self.__hoverMsg.vy = 0.0
            self.__hoverMsg.yawrate = 0.0
            self.__hoverMsg.zDistance = self._targetPoint[2]

            # Full State Messages
            self.__fullStateMsg.header.frame_id = 'world'
            self.__fullStateMsg.header.seq += 1
            self.__fullStateMsg.header.stamp = rospy.Time.now()
            self.__fullStateMsg.pose.position.x = self.__pos[0] + controlVel[0]
            self.__fullStateMsg.pose.position.y = self.__pos[1] + controlVel[1]
            self.__fullStateMsg.pose.position.z = self._targetPoint[2]

            # Services
            if self._state == "MOVING":
                goal =  np.array(
                    [
                        round(self.__pos[0] + controlVel[0], 2),
                        round(self.__pos[1] + controlVel[1], 2),
                        round(self.__pos[2] + controlVel[2], 2)
                    ]   
                )
                # logging.info(f"{self.__name} goal: {goal}")

                self.__pubFullState.publish(self.__fullStateMsg)
            elif self._state == "HOVERING":
                self.__pubHover.publish(self.__hoverMsg)

            # self.__pubSetPointPos.publish(self.__PointMsg)

            # Log info
            # logging.info(f"{self.__name} pos: {self.__pos} target: {self._targetPoint} controlVel: {controlVel}")

        return retValue

    def takeOffAsync(self, height: float) -> bool:
        """Takes off the agent from the ground. Does not block the flow of the program.

        Args:
            height (float): Target height to be taken off to.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False
        
        try:
            self.__crazyflie.takeoff(targetHeight=height, duration=2.0)
            self._state = "TAKING_OFF"
            retValue = True
        except Exception as e:
            print(e.with_traceback())
    
        return retValue
    
    def landAsync(self) -> bool:
        """Lands the agent from the ground. Does not block the flow of the program.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False
        try:
            self.__crazyflie.land(0.0, duration=3.0)
            self._state = "LANDING"
            retValue = True
        except Exception as e:
            print(e.with_traceback())
        return retValue
    
    def goToAsync(self, x: float, y: float, z: float) -> bool:
        """Moves the agent to the desired point. Does not block the flow of the program.

        Args:
            x (float): X value of the point.
            y (float): Y value of the point
            z (float): Z value of the point.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False

        self.__crazyflie.goTo(
                goal=[x, y, z],
                yaw=0.0,
                duration=4.0,
                relative=False
            )
        self._state = "MOVING"
        
        return retValue
    
    def goToSync(self, x: float, y: float, z: float) -> bool:
        """Moves the agent to the desired point. Blocks the flow of the program.

        Args:
            x (float): X value of the point.
            y (float): Y value of the point
            z (float): Z value of the point.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False

        return retValue

    def kill(self) -> bool:
        """Stops all the agen motors.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False

        for i in range(5):
            self.__pubStop.publish(Empty())

        return retValue
    
    