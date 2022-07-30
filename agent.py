"Control each agents"
import sys
import logging
import time
import numpy as np
from threading import Thread, Lock, Event
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
import settings

class Agent:
    """This class represents the real-world agent.
    It does swarming and other operations to control the agent.
    """
    __cf: Crazyflie
    __deck_attached_event: Event
    __mc: MotionCommander
    __is_ready: bool

    __uri: str
    __name: str
    __index: int
    __initial_pos: np.ndarray
    __pos: np.ndarray
    __vel: np.ndarray
    __speed: float
    __max_speed: float
    # Swarm related
    __other_agents: list

    __is_formation_active: bool
    __is_avoidance_active: bool
    __is_trajectory_active: bool
    __is_swarming: bool
    __is_in_formation: bool

    __formation_matrix: np.ndarray
    __swarm_heading: np.ndarray
    __swarm_desired_heading: np.ndarray
    __swarm_min_distance: float
    __angle_offset: float
    __target_point: np.ndarray
    __target_height: float

        # Constants
    __formation_const: float
    __trajectory_const: float

    __alpha: float # Overall force multiplier (more ALPHA means more aggressive behaviour)
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
    
    def __init__(self, uri: str, name: str, idx: int) -> None:
        """Initializes the agent class.

        Args:
            name (str): Name of the agent.
            address (str): Address of the agent.
        """
        self.__uri = uri
        self.__name = name
        self.__index = idx

        self.__is_ready = False
        self._lg_stab = None

        self.__is_formation_active = False
        self.__is_avoidance_active = False
        self.__is_trajectory_active = False
        self.__is_swarming = False
        self.__is_in_formation = False
        self.__is_static = False
        self.__formation_matrix = np.array([0.0])
        self.__swarm_min_distance = 0.30
        self.__rotation_angle = 0.0
        self.__angle_offset = 0.0

        self.__target_point = np.array([0.0, 0.0, 0.0])
        self.__target_height = 0.0
        
        self.__initial_pos = None
        self.__pos = np.array([0.0, 0.0, 0.0])
        self.__vel = np.array([0.0, 0.0, 0.0])
        self.__speed = 0.0
        self.__max_speed = 1.0
        self.__formation_const = 0.5
        self.__trajectory_const = 1.5
        self.__alpha = 1.15
        self.__beta = 2.21

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
            logging.info(f"[{self.get_name()}] Started the daemon update thread")
        except:
            logging.info(f"[{self.get_name()}] Failed to start the daemon update thread")

    def formation_control(self) -> np.ndarray:
        """Calculates the formation 'force' to be applied to the agent.
        This calculation moves the agent into formation.

        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        ret_value = np.array([0.0, 0.0, 0.0])

        # Itarete over agents and calculate the desired vectors
        for i, agent in enumerate(self.get_other_agents()):
            if agent != self:
                dist_diff = agent.get_pos() - self.get_pos()
                self_idx = self.get_index()
                dist_desired = self.get_formation_matrix()[self_idx][i]
                # print(f"[{self.getName()}] {distanceToDesiredPoint.round(4)}")
                
                rot_angle = self.get_rotation_angle()
                rot_matrix = settings.get_rotation_matrix(rot_angle)

                ret_value += (dist_diff - np.dot(rot_matrix, dist_desired))         

        return ret_value * self.get_formation_const()

    
    def avoidance_control(self) -> np.ndarray:
        """Calculates the avoidance 'force' to be applied to the agent.
        This calculation keeps the agent away from the obstacles.

        Args:
            agents (list): List of agents.

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        ret_value = np.array([0.0, 0.0, 0.0])

        for other_agent in self.get_other_agents():
            if other_agent is not self:
                dist_diff_scalar = settings.get_distance(other_agent.get_pos(), self.get_pos())

                # Check if othe agent is too close
                if dist_diff_scalar < self.__swarm_min_distance:
                    dist_diff = other_agent.get_pos() - self.get_pos()

                    # repellentVelocity that 'pushes' the agent away from the other agent
                    repellent_velocity = dist_diff / np.linalg.norm(dist_diff)
                    repellent_force = self.__alpha * (
                        pow(np.e, -(self.__beta * dist_diff_scalar) - pow(np.e, -(self.__beta * self.__swarm_min_distance)))
                    )
                    ret_value += repellent_velocity * (-repellent_force)

        return ret_value
    
    def trajectory_control(self) -> np.ndarray:
        """Calculates the trajectory 'force' to be applied to the agent.
        This calculation moves the agent towards the target point.

        Args:
            agents (list): List of agents

        Returns:
            np.ndarray: Calculated force. (Vector3)
        """
        ret_value = np.array([0.0, 0.0, 0.0])

        swarm_center = np.array([0.0, 0.0, 0.0])

        if self.__is_swarming:
            for other_agent in self.get_other_agents():
                swarm_center += other_agent.get_pos()
            swarm_center /= len(self.get_other_agents())

            ret_value = self.get_target_point() - swarm_center
        else:
            ret_value = self.get_target_point() - self.get_pos()

        return ret_value * self.__trajectory_const
    
    

    def get_index(self) -> int:
        """Agent index.

        Returns:
            int: _description_
        """
        self.__lock.acquire()
        index = self.__index
        self.__lock.release()

        return index

    def get_name(self) -> str:
        """Agent name.

        Returns:
            str: _description_
        """
        self.__lock.acquire()
        name = self.__name
        self.__lock.release()

        return name

    def get_initial_pos(self) -> np.ndarray:
        """Agent initial pos.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        initial_pos = np.array(self.__initial_pos)
        self.__lock.release()

        return initial_pos

    def get_pos(self) -> np.ndarray:
        """Agent position.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        pos = np.array(self.__pos)
        self.__lock.release()

        return pos

    def get_vel(self) -> np.ndarray:
        """Agent velocity.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        vel = np.array(self.__vel)
        self.__lock.release()

        return vel

    def get_speed(self) -> float:
        """Agent speed.

        Returns:
            float: _description_
        """
        self.__lock.acquire()
        speed = settings.get_magnitude(self.__vel)
        self.__lock.release()

        return speed

    def get_formation_matrix(self) -> np.ndarray:
        """Agent formation matrix.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        formation = self.__formation_matrix
        self.__lock.release()

        return formation

    def get_target_point(self) -> np.ndarray:
        """Agent target point.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        target = np.array(self.__target_point)
        self.__lock.release()

        return target

    def get_target_height(self) -> float:
        """Agent target height.

        Returns:
            float: _description_
        """
        self.__lock.acquire()
        height = self.__target_height
        self.__lock.release()

        return height

    def get_max_speed(self) -> float:
        """Agent max speed.

        Returns:
            float: _description_
        """
        self.__lock.acquire()
        max_speed = self.__max_speed
        self.__lock.release()

        return max_speed

    def get_formation_const(self) -> float:
        """Agent formation const.

        Returns:
            float: _description_
        """
        self.__lock.acquire()
        formation_const = self.__formation_const
        self.__lock.release()

        return formation_const

    def get_swarm_heading(self) -> np.ndarray:
        """Agent swarm heading.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        heading = np.array(self.__swarm_heading)
        self.__lock.release()

        return heading

    def get_swarm_desired_heading(self) -> np.ndarray:
        """Agent swarm desired heading.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        desired_heading = np.array(self.__swarm_desired_heading)
        self.__lock.release()

        return desired_heading

    def get_other_agents(self) -> list:
        """Agent list of other agents.

        Returns:
            list: _description_
        """
        self.__lock.acquire()
        other_agents = self.__other_agents
        self.__lock.release()

        return other_agents

    def get_angle_offset(self) -> float:
        """Agent angle offset.

        Returns:
            float: _description_
        """
        self.__lock.acquire()
        angle_offset = self.__angle_offset
        self.__lock.release()

        return angle_offset

    def get_rotation_angle(self) -> float:
        """Agent rotation angle.

        Returns:
            float: _description_
        """
        self.__lock.acquire()
        rotation_angle = self.__rotation_angle
        self.__lock.release()

        return rotation_angle

    def is_ready(self) -> bool:
        """Agent is ready.

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        is_active = self.__is_ready
        self.__lock.release()

        return is_active

    def is_formation_active(self) -> bool:
        """Agent is formation active.

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        is_active = self.__is_formation_active
        self.__lock.release()

        return is_active

    def is_trajectory_active(self) -> bool:
        """Agent is trajectory active.

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        is_active = self.__is_trajectory_active
        self.__lock.release()

        return is_active

    def is_avoidance_active(self) -> bool:
        """Agent is avoidance active.

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        is_active = self.__is_avoidance_active
        self.__lock.release()

        return is_active

    def is_in_formation(self) -> bool:
        """Agent is in formation.

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        is_in_formation = self.__is_in_formation
        self.__lock.release()

        return is_in_formation

    def is_swarming(self) -> bool:
        """Agent is swarming.

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        is_swarming = self.__is_swarming
        self.__lock.release()

        return is_swarming

    def set_index(self, index) -> int:
        """Agent set index.

        Args:
            index (_type_): _description_

        Returns:
            int: _description_
        """
        self.__lock.acquire()
        self.__index = index
        self.__lock.release()

        return True
    
    def set_is_ready(self, status: bool) -> bool:
        """Agent set is ready.

        Args:
            status (bool): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__is_ready = status
        self.__lock.release()

        return True

    def set_is_in_formation(self, status: bool) -> bool:
        """Agent set is in formation.

        Args:
            status (bool): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__is_in_formation = status
        self.__lock.release()

        return True

    def set_formation_const(self, formation_const: float) -> bool:
        """Agent set formation const

        Args:
            formation_const (float): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        old = self.__formation_const
        self.__formation_const = formation_const
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Formation const set to: {formation_const}, was: {old}")

        return True

    def set_target_point(self, target: np.ndarray) -> bool:
        """Agent set target point.

        Args:
            target (np.ndarray): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__target_point = np.array(target)
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Target x: {round(target[0], 2)} y: {round(target[1], 2)} z: {round(target[2], 2)}")

        return True

    def set_target_height(self, target_height: float) -> bool:
        """Agent set target height.

        Args:
            target_height (float): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__target_height = target_height
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Target height set to: {round(target_height, 2)}")

        return True

    def set_max_speed(self, max_speed: float) -> bool:
        """Agent set max speed.

        Args:
            max_speed (float): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__max_speed = max_speed
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Max speed set to: {round(max_speed, 2)}")

        return True

    def set_formation_active(self, status: bool) -> bool:
        """Agent formation active.

        Args:
            status (bool): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__is_formation_active = status
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Formation active set to: {status}")

        return True

    def set_avoidance_active(self, status: bool) -> bool:
        """Agent set avoidance active.

        Args:
            status (bool): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__is_avoidance_active = status
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Avoidance active set to: {status}")

        return True

    def set_trajectory_active(self, status: bool) -> bool:
        """Agent set trajectory active.

        Args:
            status (bool): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__is_trajectory_active = status
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Trajectory active set to: {status}")

        return True

    def set_formation_matrix(self, matrix: np.ndarray) -> bool:
        """Agent set formation martix.

        Args:
            matrix (np.ndarray): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__formation_matrix = np.array(matrix)
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Formation matrix has been changed: {matrix.shape}")

        return True

    def set_swarming(self, swarming: bool) -> bool:
        """Agent set swarming.

        Args:
            swarming (bool): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__is_swarming = swarming
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Swarming set to: {swarming}")

        return True

    def set_swarm_heading(self, heading: np.ndarray) -> bool:
        """Agent set swarm heading.
                logger.warning(str(e))
        Args:
            heading (np.ndarray): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__swarm_heading = np.array(heading)
        self.__lock.release()

        return True

    def set_swarm_desired_heading(self, desired_heading: np.ndarray) -> bool:
        """Agent set swarm desired heading.

        Args:
            desired_heading (np.ndarray): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__swarm_desired_heading = desired_heading
        self.__lock.release()
        
        return True
    def is_static(self) -> bool:
        """Agent is static.

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        is_static = self.__is_static
        self.__lock.release()

        return is_static
    def set_is_static(self, status: bool) -> bool:
        """Agent set is static

        Args:
            status (bool): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__is_static = status
        self.__lock.release()

        return True
    def set_obstacles(self, obstacles: list) -> bool:
        """Agent set obstacles.

        Args:
            obstacles (list): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__obstacles = obstacles
        self.__lock.release()

        return True
    def set_angle_offset(self, angle_offset: float) -> bool:
        """Agent set angle offset.

        Args:
            angle_offset (float): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__angle_offset = angle_offset
        self.__lock.release()
        
        return True

    def set_rotation(self, degree: float) -> bool:
        """Agent set rotation.

        Args:
            degree (float): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__swarm_desired_heading = np.dot(settings.get_rotation_matrix(degree), self.__swarm_heading)
        self.__lock.release()

        logging.info(f"[{self.get_name()}] Rotation set to: {round(degree, 2)}")

        return True

    def set_rotation_angle(self, rotation_angle) -> bool:
        """Agent set rotation angle

        Args:
            rotation_angle (_type_): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__rotation_angle = rotation_angle
        self.__lock.release()
        
        return True

    def set_initial_pos(self, initial_pos: np.ndarray) -> bool:
        """Agent set initial position.

        Args:
            initial_pos (np.ndarray): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__initial_pos = initial_pos
        self.__lock.release()
        
        return True

    def set_pos(self, pos: np.ndarray) -> bool:
        """Agent set posiiton.

        Args:
            pos (np.ndarray): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__pos = pos
        self.__lock.release()

        return True
    
    def set_vel(self, vel: np.ndarray) -> bool:
        """Agent set velocity

        Args:
            vel (np.ndarray): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__vel = vel
        self.__lock.release()

        return True
    
    def set_other_agents(self, other_agents: list) -> bool:
        """Agent set other agents.

        Args:
            other_agents (list): _description_

        Returns:
            bool: _description_
        """
        self.__lock.acquire()
        self.__other_agents = other_agents
        self.__lock.release()

        return True


    def update(self) -> bool:
        """Depending on the settings, calculates the control values and applies them.

        Args:
            agents (list): List of all agents in the system.

        Returns:
            bool: Whether the update was succesfull or not.
		"""
        ret_value = False
        
        self.__deck_attached_event = Event()
        
        #with SyncCrazyflie(self.__uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=self.param_deck_flow)
        self.__cf = Crazyflie(rw_cache="./cache")
        # Update __scf
        # Connect some callbacks from the Crazyflie API
        self.__cf.connected.add_callback(self.connected)
        self.__cf.disconnected.add_callback(self.disconnected)
        self.__cf.connection_failed.add_callback(self.connection_failed)
        self.__cf.connection_lost.add_callback(self.connection_lost)

        self.__cf.open_link(self.__uri)
        # with MotionCommander(scf, default_height=0.01) as mc:
        while self.__thread.is_alive():

            # Wait for agent to be ready
            if not self.is_ready():
                continue

            # Calculate control values
            control_vel = np.array([0.0, 0.0, 0.0])
            formation_vel = np.array([0.0, 0.0, 0.0])
            avoidance_vel = np.array([0.0, 0.0, 0.0])
            trajectory_vel = np.array([0.0, 0.0, 0.0])

            if self.is_formation_active():
                formation_vel = self.formation_control()

            if self.is_avoidance_active():
                avoidance_vel = self.avoidance_control()
                if 0.02 < settings.get_magnitude(avoidance_vel):
                    logging.info(f"[{self.get_name()}] Possible crash avoidance active!")

            if self.is_trajectory_active():
                trajectory_vel = self.trajectory_control()

                # Limit swarm control values
                if self.get_max_speed() < settings.get_magnitude(trajectory_vel):
                    trajectory_vel = settings.set_magnitude(trajectory_vel, self.get_max_speed())
        
            # ---- Final velocity ---- #
            control_vel = formation_vel + avoidance_vel + trajectory_vel
            # ------------------------ #

            ## UNCOMMENT THIS TO SEE CONTROL VELOCITIES
            # logging.info(f"[{self.get_name()}] ControlVel: {control_vel.round(2)}")

            # Send the commanding message
            # print(f"[{self.getName()}] {formationVel.round(4)}")
            if self.is_formation_active() or self.is_avoidance_active() or self.is_trajectory_active():
                self.__cf.commander.send_velocity_world_setpoint(control_vel[0], control_vel[1], control_vel[2], 0.0)
            time.sleep(1 / 50)

        return ret_value

    def param_deck_flow(self, _, value_str):
        """ Connection to the flow deck.
        """
        value = int(value_str)
        if value:
            self.__deck_attached_event.set()
            logging.info(f"[{self.get_name()}] Deck is attached!")
        else:
            logging.info(f"[{self.get_name()}] Deck is NOT attached!")


    def stab_log_error(self, logconf, msg) -> None:
        """ Error connecting to the agent.
        """
        logging.info(f"[{self.get_name()}] Error when logging {logconf.name, msg}")

    
    def connected(self, link_uri: str):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        logging.info(f"[{self.get_name()}] Connected to the agent! {link_uri}")

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self.__cf.log.add_config(self._lg_stab)

            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self.update_variables)

            # Start the logging
            self._lg_stab.start()

            # Set agent as ready
            self.set_is_ready(True)
        except KeyError as err:
            logging.info(f"[{self.get_name()}] Could not start log configuration, {str(err)} not found in TOC") 
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
        
    def update_variables(self, timestamp, data, logconf):
        self.set_pos(np.array(
            [
                data['stateEstimate.x'],
                data['stateEstimate.y'],
                data['stateEstimate.z']
            ]
        ))

        # Update agent info
        self.__x1 = np.array(self.__x2)
        self.__x2 = np.array(self.get_pos())

        self.__t2 = time.perf_counter()
        self.set_vel((self.__x2 - self.__x1) / (self.__t2 - self.__t1))
        self.__t1 = time.perf_counter()
        
        self.__speed = settings.get_magnitude(self.get_vel())

        # Update swarm info
        if self.__is_swarming:
            swarm_center = np.array([0.0, 0.0, 0.0])
            for agent in self.get_other_agents():
                swarm_center += agent.get_pos()
            swarm_center /= len(self.get_other_agents())

            front_agent = self.get_other_agents()[0]
            dist_diff = front_agent.get_pos() - swarm_center

            heading = dist_diff / np.linalg.norm(dist_diff)

            # Angle offset
            heading = np.dot(settings.get_rotation_matrix(self.get_angle_offset()), heading)

            self.set_swarm_heading(heading)

    def connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.set_is_ready(False)

    def connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.set_is_ready(False)

    def take_off(self, target_height: float) -> bool:
        """Takes off the agent.
        """
        ret_value = True

        self.set_target_height(target_height)
        self.set_target_point(
            np.array([
                self.get_pos()[0],
                self.get_pos()[1],
                target_height
            ])
        )

        return ret_value

    def land(self) -> bool:
        """Lands the agent.
        """
        ret_value = True

        self.set_target_height(0.0)
        self.set_target_point(
            np.array([
                self.get_pos()[0],
                self.get_pos()[1],
                0.0
            ])
        )

        return ret_value

    def kill(self) -> bool:
        """Stops all the agent motors.

        Returns:
            bool: Whether the operation was succesfull or not.
        """
        ret_value = True
        
        self.__cf.commander.send_stop_setpoint()
        return ret_value
