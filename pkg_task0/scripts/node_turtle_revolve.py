#!/usr/bin/env python


import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class turtle():

     def __init__ (self):

          self.radius = 2.35             #radius of the circle
          self.first =0                 #  to check condition to read position of turtle on starting 
          self.Initial_value =0.0       # store initial position  
          self.distance = 0                 #  track the distance 
          rospy.init_node("node_turtle_revolve")

          self.velocity_pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

          self.pose_suscriber = rospy.Subscriber('/turtle1/pose',Pose,self.     pose_callback)

          self.rate = rospy.Rate(10)
          self.value = Twist()

          self.value.linear.x = 4
          self.value.linear.y = 0
          self.value.linear.z = 0

          self.value.angular.x = 0
          self.value.angular.y = 0
          self.value.angular.z = self.value.linear.x/self.radius
          





###   function to perform action when data comes in sucribed topic ### 
     def pose_callback(self,poseMsg):
          
          self.distance = self.distance+0.054     
          rospy.loginfo("moving in circle \n %s" , self.distance)
          if(self.first==0):
               self.first =1
               self.Initial_value = poseMsg.theta
          if(poseMsg.theta<self.Initial_value and poseMsg.theta > (self.Initial_value-0.096)):
               self.value.linear.x=0
               self.value.angular.z=0
               #rospy.loginfo('Goal reached')
          

if __name__ == '__main__':
     try: 
          rotate_turtle = turtle()   
          while not rospy.is_shutdown(): 
               rotate_turtle.rate.sleep()       
               rotate_turtle.velocity_pub.publish(rotate_turtle.value)
               if(rotate_turtle.value.linear.x==0):
                    rospy.signal_shutdown("Done")
               
          rospy.loginfo('Goal reached')
          

     except rospy.ROSInitException:
          
          rospy.loginfo("node terminated")
          