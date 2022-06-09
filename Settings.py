from re import I
from cv2 import sqrt
import numpy

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

FORMATION_OFFSET = 1.0

FORMATION_TRIANGLE = None
FORMATION_SQUARE = None
FORMATION_PYRAMID = numpy.array(
    [
        [0.0, 1.0, 1.0, numpy.sqrt(2.0), numpy.sqrt(1.5)],
        [1.0, 0.0, numpy.sqrt(2.0), 1.0, numpy.sqrt(1.5)],
        [1.0, numpy.sqrt(2.0), 0.0, 1.0, numpy.sqrt(1.5)],
        [numpy.sqrt(2.0), 1.0, 1.0, 0.0, numpy.sqrt(1.5)],
        [numpy.sqrt(1.5), numpy.sqrt(1.5), numpy.sqrt(1.5), numpy.sqrt(1.5), 0.0]
    ]
) * FORMATION_OFFSET
