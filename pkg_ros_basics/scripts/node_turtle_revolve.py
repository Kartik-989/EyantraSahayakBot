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
        self.axisMovement.linear.x = 1.4 
        self.axisMovement.angular.z = 1.0
        self.theta = 0.00
        self.xCor=0.00
        self.yCor=0.00



    def getPose(self,data):
        self.theta = data.theta
        self.xCor  =data.x
        self.yCor = data.y

    def stopValue(self):
        self.stopValue = Twist()
        self.stopValue.linear.x = 0.0
        self.stopValue.angular.z = 0.0
        self.data_pub.publish(self.stopValue)


    def rotation(self):
        self.time = rospy.get_time()-self.iniTime
        self.data_pub.publish(self.axisMovement)
        #rospy.loginfo('Publishing: '+str(self.time))


if __name__ == '__main__':
    try:
       rotate_tur = Turtle()

       while not rospy.is_shutdown():
        xCor = rotate_tur.xCor
        yCor = rotate_tur.yCor
        theta  = rotate_tur.theta
        print(xCor,yCor,theta)

        rate = rospy.Rate(10)
        rate.sleep()
        rotate_tur.rotation()
        

        if(rotate_tur.theta==0.00):
            rotate_tur.stopValue()
            break




    except rospy.ROSInterruptException:
        pass
