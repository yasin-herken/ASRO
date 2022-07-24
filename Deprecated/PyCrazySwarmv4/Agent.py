import logging
import numpy as np
import Settings
import time
from threading import Thread, Lock

# Remove the word 'Sim' in order to run IRL !!!
from pycrazyswarm.crazyflie import Crazyflie

class Agent:
    """This class represents the real-world agent.
    It does swarming and other operations to control the agent.
    """
    __crazyflie: Crazyflie
    __name: str
    __index: int
    
    __pos: np.ndarray
    __vel: np.ndarray
    __speed: float

    # Swarm related
    __otherAgents: list

    __isFormationActive: bool
    __isAvoidanceActive: bool
    __isTrajectoryActive: bool
    __isSwarming: bool
    __isInFormation: bool
    __formationMatrix: np.ndarray
    __rotationAngle: float
    __swarmHeading: np.ndarray
    __swarmDesiredHeading: np.ndarray
    __swarmMinDistance: float
    __angleOffset: float

    __targetPoint: np.ndarray
    __targetHeight: float

    # Constants
    __formationConst: float
    __trajectoryConst: float

    __alpha:float # Overall force multiplier (more ALPHA means more aggressive behaviour)
    __beta: float # Logarithmic multipler (more BETA means less tolerance)
    
    # time points
    __t1: float
    __t2: float
    
    # position points
    __x1: np.ndarray
    __x2: np.ndarray

    # thread
    __thread: Thread
    __lock: Lock

    # debug
    __tp1: float
    __tp2: float
    __fps: int
    
    def __init__(self, cf: Crazyflie, name: str, idx: int) -> None:
        """Initializes the agent class.

        Args:
            cf (crazyflie): Crazyflie handler to command the realworld agent.
            name (str): Name of the agent.
            address (str): Address of the agent.
        """
        self.__crazyflie = cf
        self.__name = name
        self.__index = idx
        
        self.__isFormationActive = False
        self.__isAvoidanceActive = False
        self.__isTrajectoryActive = False
        self.__isSwarming = False
        self.__isInFormation = False
        self.__formationMatrix = np.array([0.0])
        self.__rotationAngle = 0.0
        self.__swarmHeading = np.array([0.01, 0.01, 0.0])
        self.__swarmDesiredHeading = np.array([0.0, 1.0, 0.0])
        self.__swarmMinDistance = 0.50
        self.__angleOffset = 0.0

        self.__targetPoint = np.array([0.0, 0.0, 0.0])
        self.__targetHeight = 0.0
        
        self.__pos = np.array([0.0, 0.0, 0.0])
        self.__vel = np.array([0.0, 0.0, 0.0])
        self.__speed = 0.0
        self.__maxSpeed = 0.5

        self.__formationConst = 0.15
        self.__trajectoryConst = 0.20
        self.__alpha = 0.15
        self.__beta = 1.21

        self.__t1 = time.perf_counter()
        self.__t2 = time.perf_counter()
        
        self.__x1 = np.array([0.0, 0.0, 0.0])
        self.__x2 = np.array([0.0, 0.0, 0.0])

        # debug variables
        self.__tp1 = time.perf_counter()
        self.__tp2 = time.perf_counter()
        self.__fps = 0

        # start the update thread
        self.__lock = Lock()
        self.__thread = Thread(target=self.update, daemon=True)

        try:
            self.__thread.start()
            logging.info(f"[{self.getName()}] Started the daemon update thread")
        except:
            logging.info(f"[{self.getName()}] Failed to start the daemon update thread")

    def formationControl(self) -> np.ndarray:
        """Calculates the formation 'force' to be applied to the agent.
        This calculation moves the agent into formation.
        
        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        retValue = np.array([0.0, 0.0, 0.0])

        # Itarete over agents and calculate the desired vectors
        for i, agent in enumerate(self.getOtherAgents()):
            if agent != self:
                distanceToOtherAgent = agent.getPos() - self.getPos()

                distanceToDesiredPoint = self.getFormationMatrix()[self.__index][i]
                # print(f"[{self.getName()}] {distanceToDesiredPoint.round(4)}")

                angleDiff = Settings.angleBetween(self.getSwarmHeading(), self.__swarmDesiredHeading)

                # Determine rotation direction 1.0 for counter-clockwise -1.0 for counterwise                
                if (0.5 <= angleDiff):
                    rotDir = 1.0

                    x1 = self.getSwarmHeading()[0]
                    y1 = self.getSwarmHeading()[1]

                    x2 = self.__swarmDesiredHeading[0]
                    y2 = self.__swarmDesiredHeading[1]

                    tempVal = x1 * y2 - x2 * y1

                    if tempVal <= 0.0:
                        rotDir = -1.0

                    self.__rotationAngle += rotDir * 0.1

                rotationMatrix = Settings.getRotationMatrix(self.__rotationAngle)

                retValue += (distanceToOtherAgent - np.dot(rotationMatrix, distanceToDesiredPoint))

        return retValue * self.__formationConst
    
    def avoidanceControl(self) -> np.ndarray:
        """Calculates the avoidance 'force' to be applied to the agent.
        This calculation keeps the agent away from the obstacles.

        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        retValue = np.array([0.0, 0.0, 0.0])
        
        for otherAgent in self.getOtherAgents():
            if otherAgent is not self:
                distanceToOtherAgentScaler = Settings.getDistance(otherAgent.getPos(), self.getPos())

                # Check if othe agent is too close
                if distanceToOtherAgentScaler < self.__swarmMinDistance:
                    distanceToOtherAgent = otherAgent.getPos() - self.getPos()

                    # repellentVelocity that 'pushes' the agent away from the other agent
                    repellentVelocity = distanceToOtherAgent / np.linalg.norm(distanceToOtherAgent)
                    repellentForce = self.__alpha * (
                        pow(np.e, -(self.__beta * distanceToOtherAgentScaler) - pow(np.e, -(self.__beta * self.__swarmMinDistance)))
                    )
                    retValue += repellentVelocity * (-repellentForce)

        return retValue
    
    def trajectoryControl(self) -> np.ndarray:
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
            for otherAgent in self.getOtherAgents():
                swarmCenter += otherAgent.getPos()
            swarmCenter /= len(self.getOtherAgents())
        else:
            swarmCenter = self.getPos()

        retValue = self.getTargetPoint() - swarmCenter
        
        return retValue * self.__trajectoryConst
    
    def __updateVariables(self):
        self.setPos(self.__crazyflie.position())

        # Update agent info
        self.__x1 = np.array(self.__x2)
        self.__x2 = np.array(self.getPos())

        self.__t2 = time.perf_counter()
        self.setVel((self.__x2 - self.__x1) / (self.__t2 - self.__t1))
        self.__t1 = time.perf_counter()
        
        self.__speed = Settings.getMagnitude(self.getVel())

        # Update swarm info
        if self.__isSwarming:
            swarmCenter = np.array([0.0, 0.0, 0.0])
            for agent in self.getOtherAgents():
                swarmCenter += agent.getPos()
            swarmCenter /= len(self.getOtherAgents())

            frontAgent = self.getOtherAgents()[0]
            distanceToFrontAgentFromSwarmCenter = frontAgent.getPos() - swarmCenter

            heading = distanceToFrontAgentFromSwarmCenter / np.linalg.norm(distanceToFrontAgentFromSwarmCenter)

            # Angle offset
            heading = np.dot(Settings.getRotationMatrix(self.getAngleOffset()), heading)

            self.setSwarmHeading(heading)

    def getIndex(self) -> int:
        self.__lock.acquire()
        index = self.__index
        self.__lock.release()

        return index

    def getName(self) -> str:
        self.__lock.acquire()
        name = self.__name
        self.__lock.release()

        return name
    
    def getPos(self) -> np.ndarray:
        self.__lock.acquire()
        pos = np.array(self.__pos)
        self.__lock.release()
        
        return pos
    
    def getVel(self) -> np.ndarray:
        self.__lock.acquire()
        vel = np.array(self.__vel)
        self.__lock.release()
        
        return vel
    
    def getSpeed(self) -> float:
        self.__lock.acquire()
        speed = Settings.getMagnitude(self.__vel)
        self.__lock.release()
        
        return speed

    def getFormationMatrix(self) -> np.ndarray:
        self.__lock.acquire()
        formation = self.__formationMatrix
        self.__lock.release()
        
        return formation

    def getTargetPoint(self) -> np.ndarray:
        self.__lock.acquire()
        target = np.array(self.__targetPoint)
        self.__lock.release()
        
        return target

    def getTargetHeight(self) -> float:
        self.__lock.acquire()
        height = self.__targetHeight
        self.__lock.release()
        
        return height
    
    def getMaxSpeed(self) -> float:
        self.__lock.acquire()
        maxSpeed = self.__maxSpeed
        self.__lock.release()
        
        return maxSpeed
    
    def getFormationConst(self) -> float:
        self.__lock.acquire()
        formationConst = self.__formationConst
        self.__lock.release()
        
        return formationConst
    
    def getSwarmHeading(self) -> np.ndarray:
        self.__lock.acquire()
        heading = np.array(self.__swarmHeading)
        self.__lock.release()
        
        return heading

    def getOtherAgents(self) -> list:
        self.__lock.acquire()
        otherAgents = self.__otherAgents
        self.__lock.release()
        
        return otherAgents
    
    def getAngleOffset(self) -> float:
        self.__lock.acquire()
        angleOffset = self.__angleOffset
        self.__lock.release()
        
        return angleOffset
    
    def isFormationActive(self) -> bool:
        self.__lock.acquire()
        isActive = self.__isFormationActive
        self.__lock.release()

        return isActive

    def isTrajectoryActive(self) -> bool:
        self.__lock.acquire()
        isActive = self.__isTrajectoryActive
        self.__lock.release()

        return isActive

    def isAvoidanceActive(self) -> bool:
        self.__lock.acquire()
        isActive = self.__isAvoidanceActive
        self.__lock.release()

        return isActive

    def isInFormation(self) -> bool:
        self.__lock.acquire()
        isInFormation = self.__isInFormation
        self.__lock.release()

        return isInFormation
    
    def isSwarming(self) -> bool:
        self.__lock.acquire()
        isSwarming = self.__isSwarming
        self.__lock.release()
        
        return isSwarming
    
    def setIndex(self, index) -> int:
        self.__lock.acquire()
        old = self.__index
        self.__index = index
        self.__lock.release()

        return True

    def setIsInFormation(self, status: bool) -> bool:
        self.__lock.acquire()
        self.__isInFormation = status
        self.__lock.release()

        return True

    def setFormationConst(self, formationConst: float) -> bool:
        self.__lock.acquire()
        self.__formationConst = formationConst
        self.__lock.release()
        
        return True

    def setTargetPoint(self, target: np.ndarray) -> bool:
        self.__lock.acquire()
        self.__targetPoint = np.array(target)
        self.__lock.release()

        logging.info(f"[{self.getName()}] Target point set to x:{round(target[0], 2)} y:{round(target[1], 2)} z:{round(target[2], 2)}")

        return True
    
    def setTargetHeight(self, targetHeight: float) -> bool:
        self.__lock.acquire()
        self.__targetHeight = targetHeight
        self.__lock.release()

        logging.info(f"[{self.getName()}] Target height set to: {round(targetHeight, 2)}")

        return True

    def setMaxSpeed(self, maxSpeed: float) -> bool:
        self.__lock.acquire()
        self.__maxSpeed = maxSpeed
        self.__lock.release()

        logging.info(f"[{self.getName()}] Max speed set to: {round(maxSpeed, 2)}")

        return True

    def setFormationActive(self, status: bool) -> bool:
        self.__lock.acquire()
        self.__isFormationActive = status
        self.__lock.release()

        logging.info(f"[{self.getName()}] Formation active set to: {status}")

        return True

    def setAvoidanceActive(self, status: bool) -> bool:
        self.__lock.acquire()
        self.__isAvoidanceActive = status
        self.__lock.release()

        logging.info(f"[{self.getName()}] Avoidance active set to: {status}")

        return True

    def setTrajectoryActive(self, status: bool) -> bool:
        self.__lock.acquire()
        self.__isTrajectoryActive = status
        self.__lock.release()

        logging.info(f"[{self.getName()}] Trajectory active set to: {status}")

        return True
    
    def setFormationMatrix(self, matrix: np.ndarray) -> bool:
        self.__lock.acquire()
        self.__formationMatrix = np.array(matrix)
        self.__lock.release()

        logging.info(f"[{self.getName()}] Formation matrix has been changed: {matrix.shape}")

        return True
    
    def setSwarming(self, swarming: bool) -> bool:
        self.__lock.acquire()
        self.__isSwarming = swarming
        self.__lock.release()

        logging.info(f"[{self.getName()}] Swarming set to: {swarming}")

        return True

    def setSwarmHeading(self, heading: np.ndarray) -> bool:
        self.__lock.acquire()
        self.__swarmHeading = np.array(heading)
        self.__lock.release()

        return True

    def setAngleOffset(self, angleOffset: float) -> bool:
        self.__lock.acquire()
        self.__angleOffset = angleOffset
        self.__lock.release()
        
        return True

    def setRotation(self, degree: float) -> bool:
        self.__lock.acquire()
        self.__swarmDesiredHeading = np.dot(Settings.getRotationMatrix(degree), self.__swarmHeading)
        self.__lock.release()

        logging.info(f"[{self.getName()}] Rotation set to: {round(degree, 2)}")

        return True

    def setPos(self, pos: np.ndarray) -> bool:
        self.__lock.acquire()
        self.__pos = pos
        self.__lock.release()

        return True
    
    def setVel(self, vel: np.ndarray) -> bool:
        self.__lock.acquire()
        self.__vel = vel
        self.__lock.release()

        return True
    
    def setOtherAgents(self, otherAgents: list) -> bool:
        self.__lock.acquire()
        self.__otherAgents = otherAgents
        self.__lock.release()

        return True

    def update(self) -> bool:
        """Depending on the settings, calculates the control values and applies them. 

        Args:
            agents (list): List of all agents in the system.

        Returns:
            bool: Whether the update was succesfull or not.
		"""
        retValue = False
        
        while self.__thread.is_alive():
            self.__tp2 = time.perf_counter()

            if False and 1.0 <= self.__tp2 - self.__tp1:
                logging.info(f"[{self.getName()} Updated {self.__fps} times in a second]")
                self.__tp1 = time.perf_counter()
                self.__fps = 0

            # Calculate control values
            controlVel = np.array([0.0, 0.0, 0.0])
            formationVel = np.array([0.0, 0.0, 0.0])
            avoidanceVel = np.array([0.0, 0.0, 0.0])
            trajectoryVel = np.array([0.0, 0.0, 0.0])

            # Update position, speed and swarm variables
            self.__updateVariables()

            if self.isFormationActive():
                formationVel = self.formationControl()

            if self.isAvoidanceActive():
                avoidanceVel = self.avoidanceControl()
                if 0.0 < Settings.getMagnitude(avoidanceVel):
                    logging.info(f"[{self.getName()}] Possible crash avoidance active!")

            if self.isTrajectoryActive():
                trajectoryVel = self.trajectoryControl()

                # Limit swarm control values
                if self.getMaxSpeed() < Settings.getMagnitude(trajectoryVel):
                    trajectoryVel = Settings.setMagnitude(trajectoryVel, self.getMaxSpeed())
        
            # ---- Final velocity ---- # 
            controlVel = formationVel + avoidanceVel + trajectoryVel
            # ------------------------ #

            # Send the commanding message
            # print(f"[{self.getName()}] {formationVel.round(4)}")
            self.__crazyflie.cmdVelocityWorld(controlVel, 0)

            time.sleep(1 / 50)
            self.__fps += 1

        return retValue
    
    def takeOff(self, targetHeight: float) -> bool:
        """Takes off the agent.
        """
        self.setTargetHeight(targetHeight)
        self.setTargetPoint(
            np.array([
                self.getPos()[0],
                self.getPos()[1],
                targetHeight
            ])
        )

    def land(self) -> bool:
        """Lands the agent.
        """
        self.setTargetHeight(0.0)
        self.setTargetPoint(
            np.array([
                self.getPos()[0],
                self.getPos()[1],
                0.0
            ])
        )

    def kill(self) -> bool:
        """Stops all the agent motors.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = True

        self.__crazyflie.cmdStop()

        return retValue
    
    