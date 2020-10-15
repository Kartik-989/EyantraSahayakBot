#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Turtle():
    def __init__(self):
        rospy.init_node('node_turtle_revolve', anonymous=True)  #Initializing Ros node
        
        self.data_pub=rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("turtle1/pose", Pose, self.getPose)

        self.iniTime = rospy.get_time()
        self.rate    = rospy.Rate(10)
        self.axisMovement = Twist()
        self.axisMovement.linear.x = 1.0
        self.axisMovement.linear.y = 0.0
        self.axisMovement.linear.z = 0.0

        self.axisMovement.angular.x = 0.0
        self.axisMovement.angular.y = 0.0
        self.axisMovement.angular.z = 1.0
        self.theta = 0.00


    def getPose(self,data):
        self.theta = data.theta

    def stopValue(self):
        self.stopValue = Twist()
        self.stopValue.linear.x = 0.0
        self.stopValue.linear.y = 0.0
        self.stopValue.linear.z = 0.0

        self.stopValue.angular.x = 0.0
        self.stopValue.angular.y = 0.0
        self.stopValue.angular.z = 0.0

        self.data_pub.publish(self.stopValue)
        rospy.loginfo('Goal Reached')

    def rotation(self):
        self.time = rospy.get_time()-self.iniTime
        self.data_pub.publish(self.axisMovement)
        rospy.loginfo('Moving in Circle: '+str(self.time))


if __name__ == '__main__':
    try:
       rotate_tur = Turtle()

       while not rospy.is_shutdown():
        rate = rospy.Rate(10)
        rate.sleep()
        rotate_tur.rotation()
        if(rotate_tur.theta<0.0 and rotate_tur.theta>-0.15):
            rotate_tur.stopValue()
            break

    except rospy.ROSInterruptException:
        pass
