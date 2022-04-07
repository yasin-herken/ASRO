import numpy
import datetime

from Constants import State
from Constants import Mission
from Constants import MissionInfo
from AgentController import AgentController

class Agent:
    __id: int
    __name: str
    __controller: AgentController
    __position: numpy.ndarray
    __velocity: numpy.ndarray
    __acceleration: numpy.ndarray
    __heading: numpy.ndarray
    __rotation: numpy.ndarray
    __state: State
    __mission: Mission
    __missionInfo: MissionInfo

    def __init__(self, id) -> None:
        self.__id = id
        self.__name = f"Agent_{id}"
        self.__controller = AgentController(id)
        self.__position = numpy.array([0.0, 0.0, 0.0])
        self.__velocity = numpy.array([0.0, 0.0, 0.0])
        self.__acceleration = numpy.array([0.0, 0.0, 0.0])
        self.__heading = numpy.array([0.0, 1.0, 0.0])
        self.__rotation = numpy.array([0.0, 0.0, 0.0])
        self.__state = State.STATIONARY
        self.__mission = Mission.NONE
        self.__missionInfo = None

    def __str__(self) -> str:
        agentStr = f"[{datetime.datetime.now()}] Name: {self.__name}, Pos: {self.__position}, Vel: {self.__velocity}, Acc: {self.__acceleration}, {self.__state}"

        return agentStr

    def updateMission(self, mission: Mission, missionInfo: MissionInfo):
        self.__mission = mission
        self.__missionInfo = missionInfo

    def tick(self, deltaTime):
        # update agent properties
        self.__position = self.__controller.getPosition()
        self.__velocity = self.__controller.getVelocity()
        self.__acceleration = self.__controller.getAcceleration()
        self.__heading = self.__controller.getHeading()
        self.__rotation = self.__controller.getRotation()

        # check mission
        if self.__mission == Mission.NONE:
            pass
        elif self.__mission == Mission.FORMING:
            pass
        elif self.__mission == Mission.ROTATING:
            pass
        elif self.__mission == Mission.MOVING:
            pass

        # request desired values
        self.__controller.reqVelocity(numpy.array([0.0, 0.0, 0.1]))

    def __calculateFormation(self):
        pass

    def __calculateAvoidance(self):
        pass

    def __calculateTarget(self):
        pass
    