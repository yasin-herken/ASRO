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

def getDistance(vector1: numpy.ndarray, vector2: numpy.ndarray):
    return numpy.linalg.norm(vector1 - vector2)

def getRotationMatrix(degree) -> numpy.ndarray:
    radian = degree * (numpy.pi / 180.0)

    return numpy.array(
        [
            [numpy.cos(radian), -numpy.sin(radian), 0.0],
            [numpy.sin(radian), numpy.cos(radian), 0.0],
            [0.0, 0.0, 1.0]
        ]
    )

def angleBetween(vec1, vec2) -> float:
    inRadian = numpy.arccos(numpy.clip(numpy.dot(vec1, vec2), -1.0, 1.0))
    return numpy.degrees(inRadian)

def vec3(x = 0.0, y = 0.0, z = 0.0) -> numpy.ndarray:
    return numpy.array([x, y, z])

ALPHA = 0.15 # Overall force multiplier (more ALPHA means more aggressive behaviour)
BETA = 1.21 # Logarithmic multipler (more BETA means less tolerance)

FORMATION_OFFSET = 2.0

# 2D - This is wrong do not use this!
FORMATION_TRIANGLE = numpy.array(
    [
        [0.0, 1.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
    ]
) * FORMATION_OFFSET

FORMATION_SQUARE = numpy.array(
    [
        [0.0, 1.0, numpy.sqrt(2.0), 1.0],
        [1.0, 0.0, 1.0, numpy.sqrt(2.0)],
        [numpy.sqrt(2.0), 1.0, 0.0, 1.0],
        [1.0, numpy.sqrt(2.0), 1.0, 0.0],
    ]
)* FORMATION_OFFSET

FORMATION_PYRAMID = numpy.array(
    [
        [0.0, 1.0, 1.0, numpy.sqrt(2.0), numpy.sqrt(1.5)],
        [1.0, 0.0, numpy.sqrt(2.0), 1.0, numpy.sqrt(1.5)],
        [1.0, numpy.sqrt(2.0), 0.0, 1.0, numpy.sqrt(1.5)],
        [numpy.sqrt(2.0), 1.0, 1.0, 0.0, numpy.sqrt(1.5)],
        [numpy.sqrt(1.5), numpy.sqrt(1.5), numpy.sqrt(1.5), numpy.sqrt(1.5), 0.0]
    ]
) * FORMATION_OFFSET

FORMATION_HEXAGON = numpy.array(
    [
        [0.0, 1.0, numpy.sqrt(3), 2.0, numpy.sqrt(3), 1.0],
        [1.0, 0.0, 1.0, numpy.sqrt(3), 2.0, numpy.sqrt(3)],
        [numpy.sqrt(3), 1.0, 0.0, 1.0, numpy.sqrt(3), 2.0],
        [2.0, numpy.sqrt(3), 1.0, 0.0, 1.0, numpy.sqrt(3)],
        [numpy.sqrt(3), 2.0, numpy.sqrt(3), 1.0, 0.0, 1.0],
        [1.0, numpy.sqrt(3), 2.0, numpy.sqrt(3), 1.0, 0.0],
    ]
) * FORMATION_OFFSET

# 3D - Better, stronger and harder
def pyramid() -> numpy.ndarray:
    
    zero_one = vec3(0.0, 1.0, 0.0) * FORMATION_OFFSET
    zero_two = vec3(1.0, 1.0, 0.0) * FORMATION_OFFSET
    zero_three = vec3(1.0, 0.0, 0.0) * FORMATION_OFFSET
    zero_four = vec3(0.5, 0.5, 0.5) * FORMATION_OFFSET

    one_two = vec3(1.0, 0.0, 0.0) * FORMATION_OFFSET
    one_three = vec3(1.0, -1.0, 0.0) * FORMATION_OFFSET
    one_four = vec3(0.5, -0.5, 0.5) * FORMATION_OFFSET

    two_three = vec3(0.0, -1.0, 0.0) * FORMATION_OFFSET
    two_four = vec3(-0.5, -0.5, 0.5) * FORMATION_OFFSET

    three_four = vec3(-0.5, 0.5, 0.5) * FORMATION_OFFSET

    ret_value = numpy.array(
        [
            [vec3(), zero_one, zero_two, zero_three, zero_four],
            [-zero_one, vec3(), one_two, one_three, one_four],
            [-zero_two, -one_two, vec3(), two_three, two_four],
            [-zero_three, -one_three, -two_three, vec3(), two_four],
            [-zero_four, -one_four, -two_four, -three_four, vec3()],
        ]
    )
    return ret_value
