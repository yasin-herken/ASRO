import rospy
from custom_msg.msg import general_parameters
def parse_data(team_name,uav_name):
        
        uav_inf = general_parameters()
        uav_inf.team_name = team_name
        uav_inf.uav_name = uav_name
        uav_inf.rpy.roll = 1.0
        uav_inf.rpy.pitch = 1.1
        uav_inf.rpy.yaw = 2.3
        uav_inf.pose.x =0.1
        uav_inf.pose.y =0.1
        uav_inf.pose.z =0.1
        return uav_inf

def callback(msg):
    print(msg)
if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)
    team_name = "YILDIZLARARASI"
    topic_name = "general_parameters/" +team_name 
    pub_stabilizer = rospy.Publisher(str(topic_name), general_parameters, queue_size=10) 
    rospy.Subscriber(str(topic_name),general_parameters,callback)
    # declares that node is publishing to the team_name topic using the custom message
    rate = rospy.Rate(10) #10hz go through the loop 10 times per second
    uav_inf = parse_data(str(team_name),str("E7")) #parse data which receives from
    while True:
        pub_stabilizer.publish(uav_inf)