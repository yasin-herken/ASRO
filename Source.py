from numpy import take
import rospy
import geometry_msgs.msg
import mav_msgs.msg

lastPose = geometry_msgs.msg.Pose()

def takeoff():
    print("Taking off...")
    pub = rospy.Publisher('chatter', mav_msgs.msg, 10)   

def callback(data):
    global lastPose
    lastPose = data
    
def listener():
    rospy.init_node('ASRO_Source', anonymous=True)
    rospy.Subscriber("/crazyflie2_1/pose", geometry_msgs.msg.Pose, callback)

if __name__ == '__main__':
    listener()

    while not rospy.is_shutdown():
        temp = input()
        
        if temp == "takeoff":
            takeoff()
