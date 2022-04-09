from re import T
import numpy
import datetime

from Constants import State
from Constants import Mission
from Constants import MissionInfo
from Constants import getMagnitude
from Constants import setMagnitude
from Constants import getDistance
from Constants import getRotationMatrix
from Constants import ALPHA, BETA
from Constants import FORMATION_CONTROL_CONSTANT
from Constants import TRAJECTORY_CONTROL_CONSTANT
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

    __validityCount: int

    def __init__(self, id) -> None:
        self.__id = id
        self.__name = f"Agent_{id}"
        self.__controller = AgentController( self.__id, self.__name)
        self.__position = numpy.array([0.0, 0.0, 0.0])
        self.__velocity = numpy.array([0.0, 0.0, 0.0])
        self.__acceleration = numpy.array([0.0, 0.0, 0.0])
        self.__heading = numpy.array([0.0, 1.0, 0.0])
        self.__rotation = numpy.array([0.0, 0.0, 0.0])
        self.__state = State.STATIONARY
        self.__mission = Mission.NONE
        self.__missionInfo = None
        self.__validityCount = 0

    def __str__(self) -> str:
        agentStr = f"[{datetime.datetime.now()}] Name: {self.__name}, Pos: {self.__position}, Vel: {self.__velocity}, Acc: {self.__acceleration}, {self.__state}"

        return agentStr

    def updateMission(self, mission: Mission, missionInfo: MissionInfo):
        self.__mission = mission
        self.__missionInfo = missionInfo

    def getMission(self) -> Mission:
        return self.__mission

    def getState(self) -> State:
        return self.__state

    def getPosition(self) -> numpy.ndarray:
        return self.__position

    def getId(self) -> int:
        return self.__id

    def tick(self, deltaTime):
        # update agent properties
        self.__position = self.__controller.getPosition()
        self.__velocity = self.__controller.getVelocity(deltaTime)
        self.__acceleration = self.__controller.getAcceleration()
        self.__heading = self.__controller.getHeading()
        self.__rotation = self.__controller.getRotation()

        # check mission
        if self.__mission == Mission.NONE:
            pass
        elif self.__mission == Mission.TAKE_OFF:
            # Turn on the engine
            if self.__state == State.STATIONARY:
                self.__state = State.IDLING
            # Take off
            elif self.__state == State.IDLING:
                retValue = self.__controller.reqTakeOff()
                if retValue:
                    self.__state = State.TAKING_OFF
            # Make sure take off is completed
            elif self.__state == State.TAKING_OFF:
                # print(f"{self.__name} {getMagnitude(self.__velocity)}")
                if getMagnitude(self.__velocity) <= 0.025:
                    self.__validityCount += 1
                    if 60 <= self.__validityCount:
                        self.__state = State.HOVERING
                        self.__validityCount = 0

        elif self.__mission == Mission.TAKE_FORMATION:
            # Start forming
            if self.__state == State.HOVERING:
                self.__state = State.FORMING

            # Apply forming algorithms and make sure forming is done
            elif self.__state == State.FORMING:
                finalControl = numpy.array([0.0, 0.0, 0.0])

                formationControl = FORMATION_CONTROL_CONSTANT * self.__calculateFormation()
                avoidanceControl = self.__calculateAvoidance()

                finalControl = formationControl + avoidanceControl

                self.__controller.reqVelocity(finalControl)

                if getMagnitude(self.__velocity) <= 0.20:
                    self.__validityCount += 1
                    if 60 <= self.__validityCount:
                        self.__state = State.HOVERING
                        self.__validityCount = 0

        elif self.__mission == Mission.ROTATE:
            pass

        elif self.__mission == Mission.MOVE:
            pass
        
    def __calculateFormation(self):
        retValue = numpy.array([0.0, 0.0, 0.0])

        for otherAgent in self.__missionInfo.otherAgents:
            if otherAgent is not self:
                distanceToOtherAgent = otherAgent.getPosition() - self.__position

                distanceToDesiredPoint = setMagnitude(
                    distanceToOtherAgent,
                    self.__missionInfo.formationMatrix[self.__id][otherAgent.getId()]
                )
                retValue += distanceToOtherAgent - distanceToDesiredPoint
        return retValue

    def __calculateAvoidance(self):
        retValue = numpy.array([0.0, 0.0, 0.0])
        
        for otherAgent in self.__missionInfo.otherAgents:
            if otherAgent is not self:
                distanceToOtherAgentScaler = getDistance(otherAgent.getPosition(), self.__position)

                # Check if othe agent is too close
                if distanceToOtherAgentScaler < self.__missionInfo.minimumSafeDistance:
                    distanceToOtherAgent = otherAgent.getPosition() - self.__position

                    # repellentVelocity that 'pushes' the agent away from the other agent
                    repellentVelocity = distanceToOtherAgent / numpy.linalg.norm(distanceToOtherAgent)
                    repellentForce = ALPHA * (
                        pow(numpy.e, -(BETA*distanceToOtherAgentScaler) - pow(numpy.e, -(BETA*self.__missionInfo.minimumSafeDistance)))
                    )
                    retValue += repellentVelocity * (-repellentForce)
        return retValue

    def __calculateTrajectory(self):
        retValue = numpy.array([0.0, 0.0, 0.0])

        swarmCenter = numpy.array([0.0, 0.0, 0.0])

        # Calculate swarm center
        for otherAgent in self.__missionInfo.otherAgents:
            swarmCenter += otherAgent.getPosition()
        swarmCenter /= len(self.__missionInfo.otherAgents)
        
        retValue = self.__missionInfo.targetPoint - swarmCenter

        if self.__missionInfo.maxVelocity <= getMagnitude(retValue):
            retValue = setMagnitude(retValue, self.__missionInfo.maxVelocity)
            

        return retValue
    
    def kill(self):
        self.__controller.kill()