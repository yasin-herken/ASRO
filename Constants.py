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
    LANDING = 4
    ON_MISSION = 5

class Mission(enum.Enum):
    NONE = 0
    FORMING = 1
    MOVING = 2
    ROTATING = 3

class FormationShape(enum.Enum):
    POINT = 0
    LINE = 1
    TRIANGLE = 2
    SQUARE = 3

class MissionInfo:
    otherAgents: list
    formationShape: FormationShape
    minimumSafeDistance: float 
    targetPoint: numpy.ndarray
    rotateAngle: float
    angularVelocity: float
    maxVelocity: float

    def __init__(self,
        otherAgents: list,
        formationShape: FormationShape,
        minimumSafeDistance: float,
        targetPoint: numpy.array,
        rotateAngle: float,
        angularVelocity: float,
        maxVelocity: float) -> None:
        pass


MODE = Mode.PLOTTING

ALPHA = 16.0
BETA = 0.000001

FORMATION_CONTROL_CONSTANT = 1.0
TRAJECTORY_CONTROL_CONSTANT = 1.0