import numpy as np
from pycrazyswarm import crazyflie
import Settings
import time

# Remove the word 'Sim' in order to run IRL !!!
from pycrazyswarm.crazyflie import Crazyflie
from pycrazyswarm.crazyswarm_py import Crazyswarm

# TODO: Fix the private values "_" to "__"

class Agent:
    """This class represents the real-world agent.
    It does swarming and other operations to control the agent.
    """
    __name: str
    __address: str
    _status:  bool
    _state: str
    
    __pos: np.ndarray
    __vel: np.ndarray
    
    __speed: float
    __isMoving: bool
    
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
    
    def __init__(self, cf: crazyflie, name: str, address: str) -> None:
        """Initializes the agent class.

        Args:
            cf (crazyflie): Crazyflie handler to command the realworld agent.
            name (str): Name of the agent.
            address (str): Address of the agent.
        """
        self.__crazyflie = cf
        self.__name = name
        self.__address = address

        self.__maxVel = 0.5
        
        self.__isFormationActive = False
        self.__isAvoidanceActive = False
        self.__isTrajectoryActive = True
        self.__isSwarming = False
        self.__formationMatrix = np.array([0.0])

        self._targetPoint = np.array([0.0, 0.0, 0.0])
        
        self._status = False
        self._state = "STATIONARY"
        
        self.__pos = np.array([0.0, 0.0, 0.0])
        self.__vel= np.array([0.0, 0.0, 0.0])
        
        self.__speed = 0.0
        
        self.__isMoving = False
        
        self.__pitch= 0.0
        self.__yaw= 0.0
        self.__roll = 0.0

        self.__t1 = time.perf_counter()
        self.__t2 = time.perf_counter()
        
        self.__x1 = np.array([0.0, 0.0, 0.0])
        self.__x2 = np.array([0.0, 0.0, 0.0])
        
        self.__validtyCount = 0   
        
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
        self.__crazyflie.position()
        return np.array(self.__pos)
    
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
    
    def setTargetPoint(self, target) -> bool:
        self._targetPoint = target
        
        return True
    
    def setMaxVel(self, vel) -> bool:
        self.__maxVel = vel

        return True

    def setFormationControl(self, status: bool) -> bool:
        self.__isFormationActive = status

        return True
    
    def setFormationMatrix(self, matrix: np.ndarray) -> bool:
        self.__formationMatrix = matrix

        return True
    
    def setSwarming(self, swarming: bool) -> bool:
        self.__isSwarming = bool

        return True

    def isMoving(self) -> bool:
        """Check if the agent is moving or not.

        Returns:
            bool: Whether the agent is moving or not.
        """
        
        return self.__isMoving
    
    def update(self, agents: list) -> bool:
        """Retrieves the agent information and updates the member veriables.
        Depending on the settings, calculates the formation control values and applies them. 

        Args:
            agents (list): List of all agents in the system.

        Returns:
            bool: Whether the update was succesfull or not.
		"""
        retValue = False

        # Update agent info
        self.__x1 = self.__pos
        self.__pos = self.__crazyflie.position()
        self.__x2 = self.__pos
        
        self.__t2 = time.perf_counter()
        self.__vel = (self.__x2 - self.__x1) / (self.__t2 - self.__t1)
        self.__t1 = time.perf_counter()
        
        self.__speed = Settings.getMagnitude(self.__vel)

        # Calculate is moving
        if self.__speed <= 0.01:
            self.__validtyCount += 1
            if 400 <= self.__validtyCount:
                self.__isMoving = False
                self.__validtyCount = 0
            else:
                self.__isMoving = True
        else:
            self.__isMoving = True
        
        # Calculate control values
        controlVel = np.array([0.0, 0.0, 0.0])

        if self.__isFormationActive:
            controlVel += self.__formationControl(agents)

        if self.__isAvoidanceActive:
            controlVel += self.__avoidanceControl()

        if self.__isTrajectoryActive:
            controlVel += self.__trajectoryControl(agents)

        if  self._targetPoint[2] <= 0.1 and self.__pos[2] <= 0.12:
            return

        self.__crazyflie.cmdVelocityWorld(controlVel, 0)

        return retValue

    def takeOffAsync(self, Z: float) -> bool:
        """Takes off the agent from the ground. Does not block the flow of the program.

        Args:
            height (float): Target height to be taken off to.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False
        
        try:
            self.__crazyflie.takeoff(targetHeight=Z, duration=1.0)
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
            self.__crazyflie.land(targetHeight=self.__crazyflie.initialPosition[2], duration=3.0)
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
        
        return retValue
    
    