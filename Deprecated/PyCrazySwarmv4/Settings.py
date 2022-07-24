import numpy
import vg

def getMagnitude(vector: numpy.ndarray) -> numpy.float64:
    return numpy.linalg.norm(vector)

def setMagnitude(vector: numpy.ndarray, factor: float) -> numpy.ndarray:
    magnitude = numpy.linalg.norm(vector)

    if magnitude == 0.0:
        magnitude = 0.001

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

def unitVector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / numpy.linalg.norm(vector)

def angleBetween(vec1, vec2) -> float:
    vec1 = unitVector(vec1)
    vec2 = unitVector(vec2)

    vec1[2] = 0.0
    vec2[2] = 0.0

    # inRadian = numpy.arccos(numpy.clip(numpy.dot(vec1, vec2), -1.0, 1.0))
    angle = vg.angle(vec1, vec2, units='deg')

    return angle

def vec3(x = 0.00, y = 0.00, z = 0.00) -> numpy.ndarray:
    return numpy.array([x, y, z])

FORMATION_OFFSET = 0.5

FORMATION_CONTROL_GAIN = 0.15
FORMATION_TRAJECTORY_GAIN = 2.0

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
    
    zero_one = vec3(0.00, 1.00, 0.00) * FORMATION_OFFSET
    zero_two = vec3(1.00, 1.00, 0.00) * FORMATION_OFFSET
    zero_three = vec3(1.00, 0.00, 0.00) * FORMATION_OFFSET
    zero_four = vec3(0.50, 0.50, 0.50) * FORMATION_OFFSET

    one_two = vec3(1.00, 0.00, 0.00) * FORMATION_OFFSET
    one_three = vec3(1.00, -1.00, 0.00) * FORMATION_OFFSET
    one_four = vec3(0.50, -0.50, 0.50) * FORMATION_OFFSET

    two_three = vec3(0.00, -1.00, 0.00) * FORMATION_OFFSET
    two_four = vec3(-0.50, -0.50, 0.50) * FORMATION_OFFSET

    three_four = vec3(-0.50, 0.50, 0.50) * FORMATION_OFFSET

    ret_value = numpy.array(
        [
            [vec3(), zero_one, zero_two, zero_three, zero_four],
            [-zero_one, vec3(), one_two, one_three, one_four],
            [-zero_two, -one_two, vec3(), two_three, two_four],
            [-zero_three, -one_three, -two_three, vec3(), three_four],
            [-zero_four, -one_four, -two_four, -three_four, vec3()],
        ]
    )

    return ret_value

def triangle() -> numpy.ndarray:
    zero_one = vec3(1.00, 0.00, 0.00)
    zero_two = vec3(0.50, 1.00, 0.00)
    
    one_two = vec3(-0.5, 1.0, 0.0)

    ret_value = numpy.array(
        [
            [vec3(), zero_one, zero_two],
            [-zero_one, vec3(), one_two],
            [-zero_two, -one_two, vec3()]
        ]
    )

    return ret_value
def calculate_Matrix(matrix:numpy.ndarray,matrixLength:float):
    twod_list = []                                                   
    for i in range (matrixLength):                               
        new = []                 
        for j in range (matrixLength):    
            new.append([0.0,0.0,0.0])      
        twod_list.append(new)  
    for i in range(len(twod_list)):
        for j in range(len(twod_list[i])):
            if(i!=j):
                twod_list[i][j] = matrix[j] - matrix[i]
    return numpy.array(twod_list)
def nine_pyramid()->numpy.ndarray:
    retValue = numpy.array([
        [-0.5,-0.5,0.0],
        [0.5,-0.5,0.0],
        [0.5,0.5,0.0],
        [-0.5,0.5,0.0],
        [-0.5,-0.5,1.0],
        [0.5,-0.5,1.0],
        [0.5,0.5,1.0],
        [-0.5,0.5,1.0],
        [0.5+0.5*numpy.sqrt(3),0.0,0.5]
    ])* FORMATION_OFFSET
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
def cube()->numpy.ndarray:
    retValue = numpy.array([
        [-0.5,-0.5,0.0],
        [0.5,-0.5,0.0],
        [0.5,0.5,0.0],
        [-0.5,0.5,0.0],
        [-0.5,-0.5,1.0],
        [0.5,-0.5,1.0],
        [0.5,0.5,1.0],
        [-0.5,0.5,1.0]
    ])* FORMATION_OFFSET
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
def triangle_prism():
    retValue = numpy.array([
        [-0.5,0.0,0.0],
        [0.5,0.0,0.0],
        [0,0.5*numpy.sqrt(3),0.0],
        [-0.5,0.0,1.0],
        [0.5,0.0,1.0],
        [0,0.5*numpy.sqrt(3),1.0]
    ])
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
def square_pyramid():
    retValue = numpy.array([
        [-0.5,-0.5,0.0],
        [0.5,-0.5,0.0],
        [0.5,0.5,0.0],
        [-0.5,0.5,0.0],
        [0.0,0.0,0.5*numpy.sqrt(3)]
    ])* FORMATION_OFFSET
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
def square():
    retValue = numpy.array([
        [-0.5,-0.5,0.0],
        [0.5,-0.5,0.0],
        [0.5,0.5,0.0],
        [-0.5,0.5,0.0],
    ])* FORMATION_OFFSET
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
def v():
    retValue = numpy.array([
        [-1.0,0.0,0.0],
        [-0.5,1.0,0.0],
        [0.0,2.0,0.0],
        [0.5,1.0,0.0],
        [1.0,0.0,0.0]
    ])*FORMATION_OFFSET
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
def triangle()->numpy.ndarray:
    retValue = numpy.array([
        [-0.5,0.0,0.0],
        [0.5,0.0,0.0],
        [0.0,0.5*numpy.sqrt(3),0.0]
    ])*FORMATION_OFFSET
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
def crescent()->numpy.ndarray:
    retValue = numpy.array([
        [-0.5,0.0,0.0],
        [-numpy.sqrt(2)/4,numpy.sqrt(2)/4,0.0],
        [0.0,0.5,0.0],
        [numpy.sqrt(2)/4,numpy.sqrt(2)/4,0.0],
        [0.5,0.0,0.0]
    ])*FORMATION_OFFSET
    matrixLength = int(retValue.size/3)
    return calculate_Matrix(retValue,matrixLength=matrixLength)
