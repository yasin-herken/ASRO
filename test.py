from queue import Empty
from turtle import position
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
from timehelper import TimeHelper
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
def goTo(target):
    fullstateMsg.header.frame_id = 'world'
    fullstateMsg.header.seq += 1
    fullstateMsg.header.stamp = rospy.Time.now()
    fullstateMsg.pose.position.z = target
    pubFullState.publish(fullstateMsg)
    timehelper.sleep(1/100)
def rotate(angle):
    fullstateMsg.header.frame_id = 'world'
    fullstateMsg.header.seq += 1
    fullstateMsg.header.stamp = rospy.Time.now()
    fullstateMsg.pose.position.z = 1.0
    fullstateMsg.twist.angular.z = 40
    fullstateMsg.twist.linear.x = 4
    pubFullState.publish(fullstateMsg)
    timehelper.sleep(1/100)
def hover(target):
    hoverMsg.header.frame_id = 'world'
    hoverMsg.header.stamp = rospy.Time.now()
    hoverMsg.header.seq += 1
    hoverMsg.vx = 0.0
    hoverMsg.vy = 0.0
    hoverMsg.zDistance = target
    hoverMsg.yawrate +=0.1
    pubHover.publish(hoverMsg)
    timehelper.sleep(1/100)
def vel():
    fullstateMsg.header.frame_id = '/base_link'
    fullstateMsg.header.seq += 1
    fullstateMsg.header.stamp = rospy.Time.now()
    fullstateMsg.twist.linear.z += 0.5 + 0.001
    pubFullState.publish(fullstateMsg)
if __name__ == '__main__':
    rospy.init_node('ASRO')
    cf_prefix = "cf1"
    cf = crazyflie.Crazyflie("cf1", "/cf1")

    cf.setParam("commander/enHighLevel", 1)
    
    freq_pub = 50
    timehelper = TimeHelper()
    rate = rospy.Rate(freq_pub)

    # Subscribe to get the local position of the crazyflie with prefix cf_prefix
    rospy.Subscriber(cf_prefix + "/local_position" , GenericLogData , local_position_callback)
    rospy.Subscriber(cf_prefix + "/external_position" , GenericLogData , external_position_callback)


    # The publisher to be able to control in position
    pubSetpointPos = rospy.Publisher(cf_prefix +"/cmd_position", Position , queue_size=10)
    pubHover = rospy.Publisher(cf_prefix +"/cmd_hover", Hover , queue_size=10)
    pubStop = rospy.Publisher(cf_prefix +"/cmd_stop", Empty , queue_size=10)
    pubFullState = rospy.Publisher(cf_prefix +"/cmd_full_state", FullState , queue_size=10)
    pubStop = rospy.Publisher(cf_prefix +"/cmd_stop", Empty , queue_size=10)
    pubTwist= rospy.Publisher(cf_prefix +"/cmd_vel", Twist , queue_size=10)

    posMsg = Position()
    hoverMsg = Hover()
    current_position = Position()
    fullstateMsg = FullState()
    velMsg = Twist()
    # Hover usage
    try:
        while True:
            hover(1.0)
            rate.sleep()
    except KeyboardInterrupt as e:
        print("Keyboard interrupt.")
        pubStop.publish(Empty())
        print("Done!")
    
