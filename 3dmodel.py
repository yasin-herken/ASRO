
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from random import randrange, uniform
import matplotlib.animation as anim
import threading
import time
x = np.arange(-4, 4, 1)
y = np.arange(-4, 4, 1)
z = np.arange(0, 3, 1)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(1, 2, 1, projection='3d')
 

while True:
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim3d(-4,4)
    ax.set_ylim3d(-4,4)
    ax.set_zlim3d(0,3)
    random_x_1=uniform(-4, 4)
    random_y =uniform(-4,4)
    random_x_2=uniform(0,4)
    
    plt.plot(random_x_1, random_y,0, 'yo')
    plt.plot(random_x_2, random_y,0, 'yo')
    plt.plot(0,random_y,0, 'yo')
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.cla()
    #time.sleep(0.1)
    
#plt.show()