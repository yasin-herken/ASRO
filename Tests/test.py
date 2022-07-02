from queue import Empty
from turtle import position

from numpy import full
import rospy
import time
import sim_cf.crazyflie as crazyflie
import sim_cf.uav_trajectory as uav_trajectory
import math

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import FullState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

initialPosition = None

def local_position_callback(msg):
    global current_position
    global initialPosition

    current_position.x = msg.values[0]
    current_position.y = msg.values[1]
    current_position.z = msg.values[2]
    current_position.header = msg.header

    if (initialPosition == None):
        initialPosition = [current_position.x, current_position.y, current_position.z]

def external_position_callback(msg):
    global external_position
    
    external_position.x = msg.values[0]
    external_position.y = msg.values[1]
    external_position.z = msg.values[2]
    external_position.header = msg.header
def hover(prefix,target):
    hoverMsg.header.frame_id = '/base_link'
    hoverMsg.header.seq += 1
    hoverMsg.header.stamp = rospy.Time.now()
    hoverMsg.zDistance = target
    id = int(prefix[-1:])
    print(type(id))
    pubHover[id-2].publish(hoverMsg)
def full_state(thrust):
    fullstateMsg.header.frame_id = 'world'
    fullstateMsg.header.seq += 1
    fullstateMsg.header.stamp = rospy.Time.now()
    fullstateMsg.twist.linear.z += thrust
    pubFullState.publish(fullstateMsg)
if __name__ == '__main__':
    rospy.init_node('ASRO')
    cf = list()
    cf_prefixs = ["cf2","cf3","cf4","cf5","cf6"]
    for i in range(0,4):
        cf.append(crazyflie.Crazyflie("cf"+str(i+2), "/cf"+str(i+2)))

    for cf_i in cf:
        print(cf_i)
        cf_i.setParam("commander/enHighLevel", 1)
    
    freq_pub = 50

    rate = rospy.Rate(freq_pub)

    # Subscribe to get the local position of the crazyflie with prefix cf_prefix
    for cf_prefix in cf_prefixs:
        rospy.Subscriber(cf_prefix + "/local_position" , GenericLogData , local_position_callback)
        rospy.Subscriber(cf_prefix + "/external_position" , GenericLogData , external_position_callback)
    pubSetpointPos = list()
    pubHover = list()
    pubFullState = list()
    pubStop = list()
    pubTwist = list()
    # The publisher to be able to control in position
    for cf_prefix in cf_prefixs:
        pubSetpointPos.append(rospy.Publisher(cf_prefix +"/cmd_position", Position , queue_size=10))
        pubHover.append(rospy.Publisher(cf_prefix +"/cmd_hover", Hover , queue_size=10))
        pubStop.append(rospy.Publisher(cf_prefix +"/cmd_stop", Empty , queue_size=10))
        pubFullState.append(rospy.Publisher(cf_prefix +"/cmd_full_state", FullState , queue_size=10))
        pubStop.append(rospy.Publisher(cf_prefix +"/cmd_stop", Empty , queue_size=10))
        pubTwist.append(rospy.Publisher(cf_prefix +"/cmd_vel", Twist , queue_size=10))

    posMsg = Position()
    hoverMsg = Hover()
    current_position = Position()
    fullstateMsg = FullState()

    # Hover usage
    try:
        while True:
            for cf_prefix in cf_prefixs:
                hover(cf_prefix,1.0)
            time.sleep(3.0)
    except KeyboardInterrupt as e:
        print("Keyboard interrupt.")
        pubStop.publish(Empty())
        print("Done!")
    
