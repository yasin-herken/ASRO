import time
import numpy
import airsim

from airsim.types import Pose
from airsim.types import Vector3r

from datetime import datetime

from Constants import MODE
from Constants import Mode
from Constants import getMagnitude

class AgentController:
    __id: int
    __name: str
    __clientSim: airsim.MultirotorClient
    __clientReal: None

    __pos1: numpy.ndarray # previous position
    __pos2: numpy.ndarray # current position

    def __init__(self, id: int, name: str) -> None:
        self.__id = id
        self.__name = name
        self.__pos1 = numpy.array([0.0, 0.0, 0.0])
        self.__pos2 = numpy.array([0.0, 0.0, 0.0])

        # Initialze the client depending on the mode
        if MODE == Mode.PLOTTING:
            print(f"[{datetime.now()}] [AgentController] Creating a FakeAgent")
        elif MODE == Mode.SIMULATION:
            print(f"[{datetime.now()}] [AgentController] Connecting to AirSim")
            self.__clientSim = airsim.MultirotorClient()
            self.__clientSim.confirmConnection()

            vehicleList = self.__clientSim.listVehicles()
            print(vehicleList)
            # Add new agent in the Airsim if not already exists
            if not self.__name in vehicleList:
                ret_value = False

                # Change spawn point depending on the agent id
                if self.__id % 2 == 0:
                    ret_value = self.__clientSim.simAddVehicle(
                        vehicle_name=self.__name,
                        vehicle_type="SimpleFlight",
                        pose=Pose(
                            Vector3r(self.__id, 0.0, 1.5)
                        )
                    )   
                else:
                    ret_value = self.__clientSim.simAddVehicle(
                        vehicle_name=self.__name,
                        vehicle_type="SimpleFlight",
                        pose=Pose(
                            Vector3r(0.0, self.__id + 1.0, 1.5)
                        )
                    )   

                if ret_value:
                    print(f"[{datetime.now()}] [AgentController] Created an agent in AirSim: {self.__name}")
                else:
                    print(f"[{datetime.now()}] [AgentController] Failed to create an agent in AirSim: {self.__name}")
            else:
                print(f"[{datetime.now()}] [AgentController] An agent already exists in Airsim: {self.__name}")
            
            self.__clientSim.enableApiControl(
                vehicle_name=self.__name,
                is_enabled=True
            )
            self.__clientSim.armDisarm(
                vehicle_name=self.__name,
                arm=True
            )

        elif MODE == Mode.INTEGRATION:
            pass

    def getPosition(self) -> numpy.ndarray:
        retValue = None

        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            try:
                pose = self.__clientSim.simGetObjectPose(object_name=self.__name)
                retValue = numpy.array(
                    [pose.position.x_val, pose.position.y_val, pose.position.z_val]
                )

                # update the deltaX
                self.__pos1 = self.__pos2
                self.__pos2 = retValue

                # print(f"{self.__name} :{round(pose.position.x_val, 2)}, {round(pose.position.y_val, 2)}, {round(pose.position.z_val, 2)}")

            except Exception as e:
                print(e)
        elif MODE == Mode.INTEGRATION:
            pass
        return retValue

    def getVelocity(self, deltaTime) -> numpy.ndarray:
        retValue = None

        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            retValue = (self.__pos2 - self.__pos1) / deltaTime
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue

    def getAcceleration(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def getHeading(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def getRotation(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def reqVelocity(self, desiredVelocity: numpy.ndarray) -> bool:
        retValue = False

        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            self.__clientSim.cancelLastTask(vehicle_name=self.__name)
            self.__clientSim.moveByVelocityAsync(
                vehicle_name=self.__name,
                vx=desiredVelocity[0],
                vy=desiredVelocity[1],
                vz=desiredVelocity[2],
                duration=0.25,
            )
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue
        
    def reqAcceleration(self, desiredAcceleration: numpy.ndarray) -> bool:
        retValue = False
        
        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue

    def reqTakeOff(self) -> bool:
        retValue = False

        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            try:
                print(f"[{datetime.now()}] [AgentController] Taking off in AirSim: {self.__name}")
                self.__clientSim.takeoffAsync(vehicle_name=self.__name)
                retValue = True
            except Exception as e:
                print(e)
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue

    def reqLand(self) -> bool:
        retValue = False

        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            try:
                print(f"[{datetime.now()}] [AgentController] Landing in AirSim: {self.__name}")
                self.__clientSim.landAsync(vehicle_name=self.__name)
                retValue = True
            except Exception as e:
                print(e)
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue

    def kill(self):
        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            self.__clientSim.client.close()
        elif MODE == Mode.INTEGRATION:
            pass