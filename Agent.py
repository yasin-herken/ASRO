import numpy as np
from pycrazyswarm import crazyflie

# Remove the word 'Sim' in order to run IRL !!!
from pycrazyswarm.crazyflieSim import Crazyflie

class Agent:
    """This class represents the real-world agent.
    It does swarming and other operations to control the agent.
    """
    __name: str
    __address: str
    __status:  bool
    __state: str
    
    __pos: np.ndarray
    __vel: np.ndarray
    
    __speed: float
    
    __pitch: float
    __yaw: float
    __row: float
    
    __crazyflie: Crazyflie
    
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
        
        return
    
    def __formationControl(self, agents: list) -> np.ndarray:
        """Calculates the formation 'force' to be applied to the agent.
        This calculation moves the agent into formation.
        
        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        pass
    
    def __avoidanceControl(self, agents: list) -> np.ndarray:
        """Calculates the avoidance 'force' to be applied to the agent.
        This calculation keeps the agent away from the obstacles.

        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        pass
    
    def __trajectoryControl(self, agents: list, target: np.ndarray) -> np.ndarray:
        """Calculates the trajectory 'force' to be applied to the agent.
        This calculation moves the agent towards the target point.

        Args:
            agents (list): List of agents
            target (np.ndarray): Target point.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        pass

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
        return self.__address
    
    def getPos(self) -> np.ndarray:
        """Returns the position of the agent. (Vector3)

        Returns:
            np.ndarray: Position of the agent.
        """
        pass
    
    def getVel(self) -> np.ndarray:
        """Returns the velocity of the agent. (Vector3)

        Returns:
            np.ndarray: Velocity of the agent.
        """
        pass
    
    def getSpeed(self) -> float:
        """Returns the speed of the agent. (Scaler)

        Returns:
            float: Speed of the agent.
        """
        pass
    
    def getPitch(self) -> float:
        """Returns the pitch of the agent.

        Returns:
            float: Pitch of the agent.
        """
        pass
    
    def getYaw(self) -> float:
        """Returns the yaw of the agent.

        Returns:
            float: Yaw of the agent.
        """
        pass
    
    def getRow(self) -> float:
        """Returns the row of the agent.

        Returns:
            float: Row of the agent.
        """
        pass
    
    def update(self) -> bool:
        """Retrieves the agent information and updates the member veriables.
        Depending on the settings, calculates the formation control values and applies them. 

        Returns:
            bool: Whether the update was succesfull or not.
		"""
        retValue = False
        
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
            self.__crazyflie.takeoff(targetHeight=height, duration=5.0)
            retValue = True
        except Exception as e:
            print(e.with_traceback())
    
        return retValue

    def takeOffSync(self, height: float) -> bool:
        """Takes off the agent from the ground. Blocks the flow of the program.

        Args:
            height (float): Target height to be taken off to.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False
        
        return retValue
    
    def landAsync(self) -> bool:
        """Lands the agent from the ground. Does not block the flow of the program.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False
        
        return retValue
    
    def landSync(self) -> bool:
        """Lands the agent from the ground. Block the flow of the program.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        retValue = False
        
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
    
    