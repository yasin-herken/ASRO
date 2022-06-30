import logging
import numpy as np
import Settings
import time

# Remove the word 'Sim' in order to run IRL !!!
from pycrazyswarm.crazyflie import Crazyflie
from pycrazyswarm.crazyswarm_py import Crazyswarm

from Settings import ALPHA, BETA

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
    __isInFormation: bool
    __formationMatrix: np.ndarray
    __swarmHeading: np.ndarray
    __swarmDesiredHeading: np.ndarray
    __swarmMinDistance: float

    __targetPoint: np.ndarray
    __targetHeight: float
    
    # time points
    __t1: float
    __t2: float
    
    # position points
    __x1: np.ndarray
    __x2: np.ndarray
    
    def __init__(self, cf: Crazyflie, name: str) -> None:
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
        self.__isInFormation = False
        self.__formationMatrix = np.array([0.0])
        self.__swarmHeading = np.array([0.0, 0.0, 0.0])
        self.__swarmDesiredHeading = np.array([0.0, 1.0, 0.0])
        self.__swarmMinDistance = 0.15

        self.__targetPoint = np.array([0.0, 0.0, 0.0])
        self.__targetHeight = 0.5
        
        self.__state = "STATIONARY"
        
        self.__pos = np.array([0.0, 0.0, 0.0])
        self.__vel = np.array([0.0, 0.0, 0.0])
        self.__speed = 0.0
        self.__maxSpeed = 0.5

        self.__t1 = time.perf_counter()
        self.__t2 = time.perf_counter()
        
        self.__x1 = np.array([0.0, 0.0, 0.0])
        self.__x2 = np.array([0.0, 0.0, 0.0])
 

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

                angleDiff = Settings.angleBetween(self.__swarmHeading, self.__swarmDesiredHeading)
                rotationMatrix = Settings.getRotationMatrix(0.0)

                # 1.0 for clockwise -1.0 for counter-clockwise
                rotDir = 1.0
                if self.__swarmHeading[0] <= self.__swarmDesiredHeading[0]:
                    rotDir = -1.0

                if (0.5 <= abs(angleDiff)):
                    rotationMatrix = Settings.getRotationMatrix(rotDir * min(angleDiff, 5.0))
            
                retValue += (distanceToOtherAgent - np.dot(rotationMatrix, distanceToDesiredPoint))

        return retValue
    
    def __avoidanceControl(self, agents: list) -> np.ndarray:
        """Calculates the avoidance 'force' to be applied to the agent.
        This calculation keeps the agent away from the obstacles.

        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        retValue = np.array([0.0, 0.0, 0.0])
        
        for otherAgent in agents:
            if otherAgent is not self:
                distanceToOtherAgentScaler = Settings.getDistance(otherAgent.getPos(), self.__pos)

                # Check if othe agent is too close
                if distanceToOtherAgentScaler < self.__swarmMinDistance:
                    distanceToOtherAgent = otherAgent.getPos() - self.__pos

                    # repellentVelocity that 'pushes' the agent away from the other agent
                    repellentVelocity = distanceToOtherAgent / np.linalg.norm(distanceToOtherAgent)
                    repellentForce = ALPHA * (
                        pow(np.e, -(BETA*distanceToOtherAgentScaler) - pow(np.e, -(BETA*self.__swarmMinDistance)))
                    )
                    retValue += repellentVelocity * (-repellentForce)

        return retValue
    
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

        heightLimit = self.__targetHeight + 0.1
        speedLimit = self.__maxSpeed + 0.05
        
        toleranceVal = 0.05

        if (
                (height <= 0.15) and
                (0.00 <= desiredSpeed <= speedLimit) and
                (-toleranceVal <= desiredVerticalSpeed <= toleranceVal)
            ):
            retValue = "STATIONARY"
        elif (
                (height <= heightLimit) and
                (0.00 <= desiredSpeed <= toleranceVal) and
                (-toleranceVal <= desiredVerticalSpeed <= toleranceVal)
            ):
            retValue = "HOVERING"
        elif (
                (height <= heightLimit) and
                (0.00 <= desiredSpeed <= speedLimit) and
                (-toleranceVal <= desiredVerticalSpeed <= toleranceVal)
            ):
            retValue = "MOVING"
        elif False:
            logging.info(f"Unhandled state height: {round(height, 3)}, desiredSpeed: {round(desiredSpeed,3 )}, desiredVerticalSpeed: {round(desiredVerticalSpeed, 3)}")
            logging.info(f"Values in hand heightLimit: {round(heightLimit, 3)} speedLimit: {round(speedLimit, 3)} toleranceVal: {round(toleranceVal, 3)}")
            
        return retValue
    
    def __updateVariables(self, agents: list):
        self.__pos[0] = self.__crazyflie.position()[0]
        self.__pos[1] = self.__crazyflie.position()[1]
        self.__pos[2] = self.__crazyflie.position()[2]

        # Update agent info
        self.__x1 = np.array(self.__x2)
        self.__x2 = np.array(self.__pos)

        self.__t2 = time.perf_counter()
        self.__vel = (self.__x2 - self.__x1) / (self.__t2 - self.__t1)
        self.__t1 = time.perf_counter()
        
        self.__speed = Settings.getMagnitude(self.__vel)

        # Update swarm info
        if self.__isSwarming:
            swarmCenter = np.array([0.0, 0.0, 0.0])
            for agent in agents:
                swarmCenter += agent.getPos()
            swarmCenter /= len(agents)

            frontAgent = agents[0]
            distanceToFrontAgentFromSwarmCenter = frontAgent.getPos() - swarmCenter
            self.__swarmHeading = distanceToFrontAgentFromSwarmCenter / np.linalg.norm(distanceToFrontAgentFromSwarmCenter)


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
        logging.info(f"[{self.__name}] Target point set x: {round(self.__targetPoint[0], 2)} y: {round(self.__targetPoint[1], 2)} z: {round(self.__targetPoint[2], 2)}")
    
    def setTargetHeight(self, targetHeight: float) -> bool:
        self.__targetHeight = targetHeight

    def setMaxSpeed(self, maxSpeed: float) -> bool:
        self.__maxSpeed = maxSpeed

    def setFormationActive(self, status: bool) -> bool:
        self.__isFormationActive = status

    def setAvoidanceActive(self, status: bool) -> bool:
        self.__isAvoidanceActive = status

    def setTrajectoryActive(self, status: bool) -> bool:
        self.__isTrajectoryActive = status
    
    def setFormationMatrix(self, matrix: np.ndarray) -> bool:
        self.__formationMatrix = matrix
    
    def setSwarming(self, swarming: bool) -> bool:
        self.__isSwarming = swarming

    def setRotation(self, degree: float) -> bool:
        self.__swarmDesiredHeading = np.dot(Settings.getRotationMatrix(degree), self.__swarmHeading)
        
    def isInFormation(self) -> bool:
        return self.__isInFormation

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

        # Update position, speed and swarm variables
        self.__updateVariables(agents)

        if self.__isFormationActive:
            formationVel = self.__formationControl(agents)
            formationVel[2] = 0.0

            # Check if in formation
            if Settings.getMagnitude(formationVel) <= 0.01:
                self.__isInFormation = True
            else:
                self.__isInFormation = False

        if self.__isAvoidanceActive:
            avoidanceVel = self.__avoidanceControl(agents)

        if self.__isTrajectoryActive:
            trajectoryVel = self.__trajectoryControl(agents)

        # Limit swarm control values
        if self.__maxSpeed < Settings.getMagnitude(trajectoryVel):
            trajectoryVel = Settings.setMagnitude(trajectoryVel, self.__maxSpeed)

        # Update the state of the agent
        newState = self.__estimateState(trajectoryVel)
        if self.__state != newState:
            logging.info(f"[{self.__name}] Changing state {self.__state} -> {newState}")
            self.__state = newState
    

        # ---- Final velocity ---- # 
        controlVel = 0.33 * formationVel + avoidanceVel + trajectoryVel
        # ------------------------ #

        # Send the commanding message
        if self.__state == "STATIONARY":
            self.__crazyflie.cmdStop()
        else:
            self.__crazyflie.cmdVelocityWorld(controlVel, 0)

        return retValue

    def takeOff(self, height):
        self.__crazyflie.takeoff(height, 2.0)

    def kill(self) -> bool:
        """Stops all the agent motors.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = True

        self.__crazyflie.cmdStop()

        return retValue
    
    