import numpy
import datetime

from Constants import State
from Constants import Mission
from Constants import MissionInfo

class Agent:
    __id: int
    __name: str
    __position: numpy.ndarray
    __velocity: numpy.ndarray
    __acceleration: numpy.ndarray
    __state: State
    __mission: Mission
    __missionInfo: MissionInfo

    def __init__(self, id) -> None:
        self.__id = id
        self.__name = f"Agent_{id}"
        self.__position = numpy.array([0.0, 0.0, 0.0])
        self.__velocity = numpy.array([0.0, 0.0, 0.1])
        self.__acceleration = numpy.array([0.0, 0.0, 0.0])
        self.__state = State.STATIONARY
        self.__mission = Mission.NONE
        self.__missionInfo = MissionInfo()

    def __str__(self) -> str:
        agentStr = f"[{datetime.datetime.now()}] Name: {self.__name}, Pos: {self.__position}, Vel: {self.__velocity}, Acc: {self.__acceleration}, {self.__state}"

        return agentStr

    def updateMission(self, mission: Mission, missionInfo: MissionInfo):
        self.__mission = mission
        self.__missionInfo = missionInfo

    def tick(self, deltaTime):
        # previous and current velocity
        deltaV = self.__acceleration * deltaTime
        v1 = self.__velocity
        v2 = v1 + deltaV

        self.__velocity = v2

        # previous and current position
        deltaX = (deltaTime / 2) * (v1 + v2)
        x1 = self.__position
        x2 = x1 + deltaX

        self.__position = x2

        # check mission
        if self.__mission == Mission.

    def calculateFormation(self):
        pass

    def calculateAvoidance(self):
        pass

    def calculateTarget(self):
        pass
    