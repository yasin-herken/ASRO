
import rospy
import enum
from crazyflie_driver.srv import UpdateParams
from threading import Thread
from crazyflie_driver.msg._FullState import FullState
from crazyflie_driver.msg._GenericLogData import GenericLogData
from geometry_msgs.msg._PointStamped import PointStamped
from std_msgs.msg._Empty import Empty 
import time
def localPositionCallback(localPosition):
    print(f"{round(100*localPosition.values[3],3)}")
    print(f"{round(100*localPosition.values[4],3)}")
    print(f"{round(100*localPosition.values[5],3)}")
    time.sleep(1)
def externalPositionCallback(externalPosition):
    print(f"Position :{externalPosition}")
    time.sleep(2)
class State(enum.Enum):
    STATIONARY = 0
    IDLING = 1
    TAKING_OFF = 2
    HOVERING = 3
    LANDING = 4
    MOVING = 5
class Crazyflie:
    def __init__(self,prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        print(worldFrame)
        self.rate = rospy.Rate(10)

        rospy.wait_for_service(prefix + '/update_params')

        rospy.loginfo("found update_params service")

        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)

        self.setParam("kalman/resetEstimation", 1)

        self.fullStatePub=rospy.Publisher("/cf1/cmd_full_state",FullState,queue_size=5)

        self.msg=FullState()

        self.msg.header.seq=0
        self.msg.header.stamp=rospy.Time.now()
        self.msg.header.frame_id=worldFrame

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()
        
        self.local_position=rospy.Subscriber(prefix+"/local_position",GenericLogData,localPositionCallback)
        self.external_position = rospy.Subscriber(prefix+"/external_position",PointStamped,externalPositionCallback)
        self.state = State.STATIONARY

        print(self.state)

    def takeOff(self,height):
        self.State(State.IDLING)
        time.sleep(2)
        duration = int(10*height/0.4)
        self.State(State.TAKING_OFF)
        while not rospy.is_shutdown():
            
            for y in range(duration):
                self.msg.pose.position.z=y/25.0
                self.msg.header.seq+=1
                self.msg.header.stamp = rospy.Time.now()
                self.fullStatePub.publish(self.msg)
                self.rate.sleep()
            self.State(State.HOVERING)    
            for y in range(20):
                self.msg.pose.position.z = height
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.fullStatePub.publish(self.msg)
                self.rate.sleep()
            break
            
    def land(self):
        self.State(State.MOVING)
        time.sleep(2)
        self.State(State.LANDING)
        lastHeight=self.msg.pose.position.z
        while not rospy.is_shutdown():
            while lastHeight > 0:
                #self.msg.twist.linear.z-=2.0
                self.msg.pose.position.z = lastHeight
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.fullStatePub.publish(self.msg)
                self.rate.sleep()
                lastHeight -= 0.02
            self.stop_pub.publish(self.stop_msg)
            self.State(State.STATIONARY)
            break
        
    def State(self,nextState):
        self.state = nextState
        print(self.state)

    def reqtakeOff(self)->bool:
        if(self.state==State.STATIONARY):
            return True
        else:
            return False
    def reqland(self)->bool:
        if(self.state==State.HOVERING or self.state==State.MOVING):
            return True
        else:
            return False
    
    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

def handler(cf):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if(cf.reqtakeOff()):
            cf.takeOff(1.5)
            rate.sleep()
        if(cf.reqland()):
            cf.land()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('crazyflie_server')

    cf1 = Crazyflie('cf1')

    cf1.setParam("commander/enHighLevel", 1)

    r = rospy.Rate(50)

    t1 = Thread(target=handler, args=(cf1,))
    t1.start()
