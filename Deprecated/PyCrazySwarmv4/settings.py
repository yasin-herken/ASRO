"""
    Global variable and function is decleared
"""
import numpy

import vg

def get_magnitude(vector: numpy.ndarray) -> numpy.float64:
    "return magnitude of matrix"
    return numpy.linalg.norm(vector)

def set_magnitude(vector: numpy.ndarray, factor: float) -> numpy.ndarray:
    "set magnitude of matrix"
    magnitude = numpy.linalg.norm(vector)

    if magnitude == 0.0:
        magnitude = 0.001

    newX = vector[0] * factor / magnitude
    newY = vector[1] * factor / magnitude
    newZ = vector[2] * factor / magnitude

    return numpy.array([newX, newY, newZ])

def get_distance(vector1: numpy.ndarray, vector2: numpy.ndarray):
    "scaler distance between two vectors"
    return numpy.linalg.norm(vector1 - vector2)

def get_rotation_matrix(degree) -> numpy.ndarray:
    "return rotation matrix after taking degree of rotation"
    radian = degree * (numpy.pi / 180.0)

    return numpy.array(
        [
            [numpy.cos(radian), -numpy.sin(radian), 0.0],
            [numpy.sin(radian), numpy.cos(radian), 0.0],
            [0.0, 0.0, 1.0]
        ]
    )

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / numpy.linalg.norm(vector)

def angle_between(vec1, vec2) -> float:
    "return angle between two vectors"
    vec1 = unit_vector(vec1)
    vec2 = unit_vector(vec2)

    vec1[2] = 0.0
    vec2[2] = 0.0

    # inRadian = numpy.arccos(numpy.clip(numpy.dot(vec1, vec2), -1.0, 1.0))
    angle = vg.angle(vec1, vec2, units='deg')

    return angle

def vec3(x_val = 0.00, y_val = 0.00, z_val = 0.00) -> numpy.ndarray:
    "return 3d vector"
    return numpy.array([x_val, y_val, z_val])

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

def calculate_matrix(matrix:numpy.ndarray,matrix_length:float):
    "Calculation of formation matrix after getting final position of each agents"
    twod_list = []
    for i in range(matrix_length):
        new = []
        for j in range(matrix_length):
            new.append([0.0,0.0,0.0])
        twod_list.append(new)
    for i in range(matrix_length):
        for j in range(matrix_length):
            if i!=j:
                twod_list[i][j] = matrix[j] - matrix[i]
    return numpy.array(twod_list)
def nine_pyramid()->numpy.ndarray:
    "return formation matrix for the nine_pyramid shape with nine agents"
    ret_value = numpy.array([
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
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
def cube()->numpy.ndarray:
    "return formation matrix for the cube shape with 8 agents"
    ret_value = numpy.array([
        [-0.5,-0.5,0.0],
        [0.5,-0.5,0.0],
        [0.5,0.5,0.0],
        [-0.5,0.5,0.0],
        [-0.5,-0.5,1.0],
        [0.5,-0.5,1.0],
        [0.5,0.5,1.0],
        [-0.5,0.5,1.0]
    ])* FORMATION_OFFSET
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
def triangle_prism():
    "return formation matrix for the triangle prism shape with 6 agents"
    ret_value = numpy.array([
        [-0.5,0.0,0.0],
        [0.5,0.0,0.0],
        [0,0.5*numpy.sqrt(3),0.0],
        [-0.5,0.0,1.0],
        [0.5,0.0,1.0],
        [0,0.5*numpy.sqrt(3),1.0]
    ])
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
def square_pyramid():
    "return formation matrix for the square pyramid shape with 4 agents"
    ret_value = numpy.array([
        [-0.5,-0.5,0.0],
        [0.5,-0.5,0.0],
        [0.5,0.5,0.0],
        [-0.5,0.5,0.0],
        [0.0,0.0,0.5*numpy.sqrt(3)]
    ])* FORMATION_OFFSET
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
def square():
    "return formation matrix for the square(2d) shape with 4 agents"
    ret_value = numpy.array([
        [-0.5,-0.5,0.0],
        [0.5,-0.5,0.0],
        [0.5,0.5,0.0],
        [-0.5,0.5,0.0],
    ])* FORMATION_OFFSET
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
def v_shape():
    "return formation matrix for the V shape with 5 agents"
    ret_value = numpy.array([
        [-1.0,0.0,0.0],
        [-0.5,1.0,0.0],
        [0.0,2.0,0.0],
        [0.5,1.0,0.0],
        [1.0,0.0,0.0]
    ])*FORMATION_OFFSET
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
def triangle()->numpy.ndarray:
    "return formation matrix for the triangle(2d) shape with 3 agents"
    ret_value = numpy.array([
        [-0.5,0.0,0.0],
        [0.5,0.0,0.0],
        [0.0,0.5*numpy.sqrt(3),0.0]
    ])*FORMATION_OFFSET
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
def crescent()->numpy.ndarray:
    "return formation matrix for the crescent shape with 5 agents"
    ret_value = numpy.array([
        [-0.5,0.0,0.0],
        [-numpy.sqrt(2)/4,numpy.sqrt(2)/4,0.0],
        [0.0,0.5,0.0],
        [numpy.sqrt(2)/4,numpy.sqrt(2)/4,0.0],
        [0.5,0.0,0.0]
    ])*FORMATION_OFFSET
    matrix_length = int(ret_value.size/3)
    return calculate_matrix(ret_value,matrix_length=matrix_length)
if __name__=="__main__":
    print(triangle())
    