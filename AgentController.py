import time
import numpy

from Constants import MODE
from Constants import Mode

class FakeAgent:
    id: int
    name: str
    position: numpy.ndarray
    velocity: numpy.ndarray
    acceleration: numpy.ndarray
    heading: numpy.ndarray
    rotation: numpy.ndarray

    lastUpdateTime: float

    def __init__(self, id) -> None:
        self.id = id
        self.name = f"FakeAgent_{id}"
        self.position = numpy.array([0.0, 0.0, 0.0])
        self.velocity = numpy.array([0.0, 0.0, 0.0])
        self.acceleration = numpy.array([0.0, 0.0, 0.0])
        self.heading = numpy.array([0.0, 1.0, 0.0])
        self.rotation = numpy.array([0.0, 0.0, 0.0])
        self.lastUpdateTime = time.perf_counter()

    def update(self) -> None:
        currTime = time.perf_counter()
        deltaTime = currTime - self.lastUpdateTime

        # change in velocity
        deltaV = self.acceleration * deltaTime
        v1 = self.velocity
        v2 = v1 + deltaV

        self.velocity = v2

        # change in position
        deltaX = (deltaTime / 2) * (v1 + v2)
        x1 = self.position
        x2 = x1 + deltaX

        self.position = x2

        self.lastUpdateTime = time.perf_counter()

class AgentController:
    __clientFake: FakeAgent
    __clientSim: None
    __clientReal: None

    def __init__(self, id: int) -> None:

        # Set the client depending on the mode
        if MODE == Mode.PLOTTING:
            self.__clientFake = FakeAgent(id)
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def getPosition(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            return self.__clientFake.position
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def getVelocity(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            return self.__clientFake.velocity
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def getAcceleration(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            return self.__clientFake.acceleration
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def getHeading(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            return self.__clientFake.heading
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def getRotation(self) -> numpy.ndarray:
        if MODE == Mode.PLOTTING:
            return self.__clientFake.rotation
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

    def reqVelocity(self, desiredVelocity: numpy.ndarray) -> bool:
        retValue = False

        if MODE == Mode.PLOTTING:
            self.__clientFake.velocity = desiredVelocity
            self.__clientFake.update()
            retValue = True
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue
        
    def reqAcceleration(self, desiredAcceleration: numpy.ndarray) -> bool:
        retValue = False
        
        if MODE == Mode.PLOTTING:
            self.__clientFake.acceleration = desiredAcceleration
            self.__clientFake.update()
            retValue = True
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
            pass
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue

    def reqLand(self) -> bool:
        retValue = False

        if MODE == Mode.PLOTTING:
            pass
        elif MODE == Mode.SIMULATION:
            pass
        elif MODE == Mode.INTEGRATION:
            pass

        return retValue