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
    __scf: SyncCrazyflie
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
    __otherAgents: list

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

    # Callbacks
    def param_deck_flow(self, _, value_str):
        value = int(value_str)
        print(value)
        if value:
            self.__deck_attached_event.set()
            logging.info('Deck is attached!')
        else:
            logging.info('Deck is NOT attached!')
    
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
        
        self.__is_formation_active = False
        self.__is_avoidance_active = False
        self.__is_trajectory_active = True
        self.__is_swarming = False
        self.__is_in_formation = False
        self.__formation_matrix = np.array([0.0])
        self.__rotation_angle = 0.0
        self.__swarm_heading = np.array([0.0, 0.0, 0.0])
        self.__swarm_desired_heading = np.array([0.0, 1.0, 0.0])
        self.__swarm_min_distance = 0.15
        self.__angle_offset = 0.0

        self.__target_point = np.array([0.0, 0.0, 0.0])
        self.__target_height = 0.0
        
        self.__initial_pos = None
        self.__pos = np.array([0.0, 0.0, 0.0])
        self.__vel = np.array([0.0, 0.0, 0.0])
        self.__speed = 0.0
        self.__max_speed = 0.5

        self.__formation_const = 0.15
        self.__trajectory_const = 0.20
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

                dist_desired = self.get_formation_matrix()[self.__index][i]
                # print(f"[{self.getName()}] {distanceToDesiredPoint.round(4)}")

                angle_diff = settings.angle_between(self.get_swarm_heading(), self.__swarm_desired_heading)

                # Determine rotation direction
                # 1.0 for counter-clockwise -1.0 for counterwise                
                if (0.5 <= angle_diff):
                    rot_dir = 1.0

                    x_1 = self.get_swarm_heading()[0]
                    y_1 = self.get_swarm_heading()[1]

                    x_2 = self.__swarm_desired_heading[0]
                    y_2 = self.__swarm_desired_heading[1]

                    temp_val = x_1 * y_2 - x_2 * y_1

                    if temp_val <= 0.0:
                        rot_dir = -1.0

                    rot_angle = self.get_rotation_angle()
                    self.set_rotation_angle(rot_angle + rot_dir * 0.1)
                

                rot_angle = self.get_rotation_angle()
                rot_matrix = settings.get_rotation_matrix(rot_angle)

                ret_value += (dist_diff - np.dot(rot_matrix, dist_desired))

        return ret_value * self.__formation_const

    
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
        else:
            swarm_center = self.get_pos()

        ret_value = self.get_target_point() - swarm_center

        return ret_value * self.__trajectory_const
    
    def __updateVariables(self, timestamp, data, logconf):
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
    def set_is_ready(bool_value):
        """Agent formation matrix.

        Returns:
            np.ndarray: _description_
        """
        self.__lock.acquire()
        value = bool_value
        self.__lock.release()
        return value
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
except KeyError as e:
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
    def battery(self, timestamp, data, logconf):
        "Print battery charge"
        print(f"battery : {data}")
        pass
    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))
    def update(self) -> bool:
        """Depending on the settings, calculates the control values and applies them. 

        Args:
            agents (list): List of all agents in the system.

        Returns:
            bool: Whether the update was succesfull or not.
		"""
        ret_value = False

        self.__deck_attached_event = Event()

        with SyncCrazyflie(self.__uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=self.param_deck_flow)

            logconf = LogConfig(name='Position', period_in_ms=10)
            logconf.add_variable('stateEstimate.x', 'float')
            logconf.add_variable('stateEstimate.y', 'float')
            logconf.add_variable('stateEstimate.z', 'float')
            battery = LogConfig(name='Battery',period_in_ms=1000)
            battery.add_variable("pm.vbat", "float")
            try:
                scf.cf.log.add_config(logconf)
                scf.cf.log.add_config(battery)
                logconf.data_received_cb.add_callback(cb=self.__updateVariables)
                battery.data_received_cb.add_callback(cb=self.battery)
                logconf.error_cb.add_callback(self._stab_log_error)
                if not self.__deck_attached_event.wait(timeout=5):
                    logging.info('No flow deck detected!')
                    sys.exit(-4)
                
                logconf.start() 
                battery.start()
            except KeyError as e:
                logging.error(str(e))
  
            with MotionCommander(scf, default_height=0.01) as mc:
                print("asldkaşsdkaldakşs")
                while True:
                    # Calculate control values
                    control_vel = np.array([0.0, 0.0, 0.0])
                    formation_vel = np.array([0.0, 0.0, 0.0])
                    avoidance_vel = np.array([0.0, 0.0, 0.0])
                    trajectory_vel = np.array([0.0, 0.0, 0.0])

                    if self.is_formation_active():
                        formation_vel = self.formation_control()

                    if self.is_avoidance_active():
                        avoidance_vel = self.avoidance_control()
                        if 0.0 < settings.get_magnitude(avoidance_vel):
                            logging.info(f"[{self.get_name()}] Possible crash avoidance active!")

                    if self.is_trajectory_active():
                        trajectory_vel = self.trajectory_control()

                        # Limit swarm control values
                        if self.get_max_speed() < settings.get_magnitude(trajectory_vel):
                            trajectory_vel = settings.set_magnitude(trajectory_vel, self.get_max_speed())
                
                    # ---- Final velocity ---- # 
                    control_vel = formation_vel + avoidance_vel + trajectory_vel
                    # ------------------------ #

                    # Send the commanding message
                    # print(f"[{self.getName()}] {formationVel.round(4)}")
                    scf.cf.commander.send_velocity_world_setpoint(control_vel[0],control_vel[1],control_vel[2],0.0)
                    time.sleep(1 / 50)
        return ret_value

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

        self.__scf.cf.commander.send_stop_setpoint()
        return ret_value
