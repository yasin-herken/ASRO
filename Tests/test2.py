from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import FullState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import rospy
import sim_cf.crazyflie as crazyflie
from test import hover
def local_position_callback(msg):
    global current_position
    current_position.x = msg.values[0]
    current_position.y = msg.values[1]
    current_position.z = msg.values[2]
    current_position.header = msg.header

    print([current_position.x,current_position.y,current_position.z])

def external_position_callback(msg):
    global external_position
    
    external_position.x = msg.values[0]
    external_position.y = msg.values[1]
    external_position.z = msg.values[2]
    external_position.header = msg.header
    print([msg.value[0],msg.value[1],msg.value[2]])
def local_position_callback_2(msg):
    global current_position_2
    current_position_2.x = msg.values[0]
    current_position_2.y = msg.values[1]
    current_position_2.z = msg.values[2]
    current_position_2.header = msg.header

    print([current_position_2.x,current_position_2.y,current_position_2.z])

def external_position_callback_2(msg):
    global external_position_2
    
    external_position_2.x = msg.values[0]
    external_position_2.y = msg.values[1]
    external_position_2.z = msg.values[2]
    external_position._2header = msg.header
    print([external_position_2.x,external_position_2.y,external_position_2.z])

if __name__ == '__main__':
    rospy.init_node('ASRO')
    cf = crazyflie.Crazyflie("cf1","/cf1")
    cf_prefix = "/cf1"
    cf_prefix2= "/cf2"
    cf.setParam("commander/enHighLevel", 1)

    rospy.Subscriber("/cf1" + "/local_position" , GenericLogData , local_position_callback)
    rospy.Subscriber("/cf1" + "/external_position" , GenericLogData , external_position_callback)

    rospy.Subscriber("/cf2" + "/local_position" , GenericLogData , local_position_callback_2)
    rospy.Subscriber("/cf2" + "/external_position" , GenericLogData , external_position_callback_2)

    pubSetpointPos = rospy.Publisher(cf_prefix +"/cmd_position", Position , queue_size=10)
    pubHover = rospy.Publisher(cf_prefix +"/cmd_hover", Hover , queue_size=10)
    pubStop = rospy.Publisher(cf_prefix +"/cmd_stop", Empty , queue_size=10)
    pubFullState = rospy.Publisher(cf_prefix +"/cmd_full_state", FullState , queue_size=10)
    pubStop = rospy.Publisher(cf_prefix +"/cmd_stop", Empty , queue_size=10)
    pubTwist = rospy.Publisher(cf_prefix +"/cmd_vel", Twist , queue_size=10)

    pubSetpointPos2 = rospy.Publisher(cf_prefix2 +"/cmd_position", Position , queue_size=10)
    pubHover2 = rospy.Publisher(cf_prefix2 +"/cmd_hover", Hover , queue_size=10)
    pubStop2 = rospy.Publisher(cf_prefix2 +"/cmd_stop", Empty , queue_size=10)
    pubFullState2 = rospy.Publisher(cf_prefix2 +"/cmd_full_state", FullState , queue_size=10)
    pubStop2 = rospy.Publisher(cf_prefix2 +"/cmd_stop", Empty , queue_size=10)
    pubTwist2 = rospy.Publisher(cf_prefix2 +"/cmd_vel", Twist , queue_size=10)

    posMsg = Position()
    hoverMsg = Hover()
    current_position = Position()
    fullstateMsg = FullState()
    external_position = Position()
    current_position_2 = Position()
    external_position_2 = Position()
    try:
        while True:
            hoverMsg.header.frame_id = 'base_link'
            hoverMsg.header.seq += 1
            hoverMsg.header.stamp = rospy.Time.now()
            hoverMsg.zDistance = 0.75
            pubHover.publish(hoverMsg)
            pubHover2.publish(hoverMsg)
            rospy.sleep(3)
    except KeyboardInterrupt as e:
        print("Keyboard interrupt.")
        pubStop.publish(Empty())
        print("Done!")