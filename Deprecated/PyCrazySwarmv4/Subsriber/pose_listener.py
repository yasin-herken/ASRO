
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from custom_msg.msg import general_parameters #import custom_msg which is in the workspace

def callback(data):
    string_1 = "\nTeam Name: " + str(data.team_name) +"\nUav Name: "+  str(data .uav_name) 
    string_2 ="\nRoll: " + str(data.rpy.roll) + "\nPitch: " + str(data.rpy.pitch) + "\nYaw: "+str(data.rpy.roll)
    string_3 ="\nx: " + str(data.pose.x) + "\ny: " + str(data.pose.y) + "\nz: " + str(data.pose.z)  
    string_data = string_1 + string_2 + string_3
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", string_data)


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/general_parameters/YILDIZLARARASI", general_parameters, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
