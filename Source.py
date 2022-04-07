import numpy as np
import rospy
import mav_msgs.msg
import geometry_msgs.msg 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
import std_msgs
import time
import nav_msgs.msg
from geometry_msgs.msg import Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
class TBController:
    def __init__(self):
        rospy.init_node('ASRO', anonymous=True)
        self.trajectory_pub= rospy.Publisher('/crazyflie2/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
        self.traj = MultiDOFJointTrajectory()
        self.traj.header.frame_id =''
        self.traj.header.stamp = rospy.Time.now()
        self.traj.joint_names = ["base_link"]
        transforms = Transform()
        print(transforms)
        velocities =Twist()
        print(velocities)
        accelerations=Twist()
        print(accelerations)
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(0.1))
        self.traj.points.append(point)
        self.trajectory_pub.publish(self.traj)
        self.r = rospy.Rate(100)
    def callback(self,data):    
        print(data)
    def start(self):
        while not rospy.is_shutdown():
            # Controller code goes between these comments
            # Differential drive robot with forward speed and angular rate
            # END Controller code
            self.trajectory_pub.publish(self.traj)
            self.r.sleep()




if __name__ == '__main__':
    vel=TBController()
    vel.start()
        