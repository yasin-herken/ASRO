from re import I
from cv2 import sqrt
import numpy

def getMagnitude(vector: numpy.ndarray) -> numpy.float64:
    return numpy.linalg.norm(vector)

def setMagnitude(vector: numpy.ndarray, factor: float) -> numpy.ndarray:
    magnitude = numpy.linalg.norm(vector)

    if magnitude == 0.0:
        magnitude = 0.00001

    newX = vector[0] * factor / magnitude
    newY = vector[1] * factor / magnitude
    newZ = vector[2] * factor / magnitude

    return numpy.array([newX, newY, newZ])
def getDistance(pos1:numpy.ndarray,pos2:numpy.ndarray) -> float:
    distance = numpy.sqrt(((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2+(pos1[2]-pos2[2])**2))
    return distance
FORMATION_OFFSET = 1.0

FORMATION_TRIANGLE = numpy.array([
    [0.0,1.0,1.0],
    [1.0,0.0,1.0],
    [1.0,1.0,0.0]
])
FORMATION_SQUARE = None
FORMATION_PYRAMID = numpy.array(
    [
        [0.0, 0.5, numpy.sqrt(5.0)/2, numpy.sqrt(5.0)/2,1.0],
        [0.5, 0.0, numpy.sqrt(2.0)/2,1.0,numpy.sqrt(5.0)/2],
        [numpy.sqrt(5.0)/2,numpy.sqrt(2.0)/2, 0.0, numpy.sqrt(2.0)/2,numpy.sqrt(5.0)/2],
        [numpy.sqrt(5.0)/2, 1.0, numpy.sqrt(2.0)/2, 0.0, 0.5],
        [1.0, numpy.sqrt(5.0)/2, numpy.sqrt(5.0)/2,0.5, 0.0]
    ]
) * FORMATION_OFFSET

if __name__ =="__main__":
    w, h = 3, 3
    matrix = [[0 for x in range(w)] for y in range(h)] 
    for i in range(3):
        for j in range(3):
            if (i == j):
                matrix[i][j] = 0.0
            else:
                matrix[i][j] = 1.0
    print(matrix)