import enum
import numpy

class Mode(enum.Enum):
    PLOTTING = 0
    SIMULATION = 1
    INTEGRATION = 2

class State(enum.Enum):
    STATIONARY = 0
    IDLING = 1
    TAKING_OFF = 2
    HOVERING = 3
    FORMING = 4
    ROTATING = 5
    LANDING = 6
    ON_MOVE = 7

class Mission(enum.Enum):
    NONE = 0
    TAKE_OFF = 1
    TAKE_FORMATION = 2
    ROTATE = 3
    MOVE = 4

class FormationShape(enum.Enum):
    POINT = 0
    LINE = 1
    TRIANGLE = 2
    SQUARE = 3

class MissionInfo:
    otherAgents: list
    formationShape: FormationShape
    formationMatrix: numpy.ndarray
    minimumSafeDistance: float 
    targetPoint: numpy.ndarray
    rotateAngle: float
    angularVelocity: float
    oneRotateStepAngle: float
    currentRotateStep: int
    targetRotateStep: int
    maxVelocity: float

    def __init__(self,
        otherAgents: list,
        formationShape: FormationShape,
        formationMatrix: numpy.ndarray,
        minimumSafeDistance: float,
        targetPoint: numpy.ndarray,
        rotateAngle: float,
        angularVelocity: float,
        oneRotateStepAngle: float,
        currentRotateStep: int,
        targetRotateStep: int,
        maxVelocity: float) -> None:

        self.otherAgents = otherAgents
        self.formationShape = formationShape
        self.formationMatrix = formationMatrix
        self.minimumSafeDistance = minimumSafeDistance
        self.targetPoint = targetPoint
        self.rotateAngle = rotateAngle
        self.angularVelocity = angularVelocity
        self.oneRotateStepAngle = oneRotateStepAngle
        self.currentRotateStep = currentRotateStep
        self.targetRotateStep = targetRotateStep
        self.maxVelocity = maxVelocity

MODE = Mode.SIMULATION

ALPHA = 4.0
BETA = 0.0001

FORMATION_CONTROL_CONSTANT = 0.5
TRAJECTORY_CONTROL_CONSTANT = 1.0

FORMATION_TRIANGLE = numpy.array(
        [
            [0.0, 3.0, 3.0],
            [3.0, 0.0, 3.0],
            [3.0, 3.0, 0.0],
        ]
    )

def getMagnitude(vector: numpy.ndarray) -> numpy.float64:
    return numpy.linalg.norm(vector)

def setMagnitude(vector: numpy.ndarray, factor: float) -> numpy.ndarray:
    magnitude = numpy.linalg.norm(vector)

    if magnitude == 0.0:
        magnitude = 0.01

    newX = vector[0] * factor / magnitude
    newY = vector[1] * factor / magnitude
    newZ = vector[2] * factor / magnitude

    return numpy.array([newX, newY, newZ])

def getDistance(vec1, vec2) -> numpy.ndarray:
    return numpy.linalg.norm(vec2 - vec1)

def getRotationMatrix(degree) -> numpy.ndarray:
    radian = degree * (numpy.pi / 180.0)

    return numpy.array(
        [
            [numpy.cos(radian), -numpy.sin(radian), 0.0],
            [numpy.sin(radian), numpy.cos(radian), 0.0],
            [0.0, 0.0, 1.0]
        ]
    )