#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import math

class Controller():
	def __init__(self):
		rospy.init_node('ebot_controller')
		self.data_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
		rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.rate = rospy.Rate(20) 
		self.pose=[0,0,0]

		self.velocity_msg = Twist()
		self.velocity_msg.linear.x = 0
		self.velocity_msg.angular.z = 0
		self.data_pub.publish(self.velocity_msg)
		self.front=0.0
		self.right=0.0
		self.x_targ = 12.5
		self.y_targ = 0.0
		self.regions={}
		self.resolCount=0.5

########################### Waypoints calculation #############################
	def Waypoints(self,count):
		counter = 0
		x=[]
		y=[]
		while counter<=6.4: 												 #loop will continue for 2*PIE, i.e. ~6.28
			yArguments=2*(math.sin(counter))*(math.sin(counter/2))			#Value for Y is calculated using function 2(sin(x)*sin(x/2))
			x.append(counter)
			y.append(yArguments)
			counter=counter+count
		return [x,y]
###################################################################


############ function for visiting waypoints for Sin-Wave ##########
	def waypointsVisit(self,waypoints):
		self.rate.sleep()
		i=0
		while i<(len(waypoints[0])-1):
			while True:
				x_dis=waypoints[0][i+1]-self.pose[0]
				y_dis=waypoints[1][i+1]-self.pose[1]
				theta_goal =  math.atan2(y_dis,x_dis)
				e_theta = (theta_goal-self.pose[2])*16
				distance = math.sqrt(math.pow(x_dis,2)+math.pow(y_dis,2))
				self.velocity_msg.linear.x = 1	
				self.velocity_msg.angular.z = e_theta
				self.data_pub.publish(self.velocity_msg) 
				if(distance<0.1):
					
					break

			i=i+1
		
		
		
####################################################################

############### Receiving the Pose and heading values of Bot ########
	def odom_callback(self,data):
		x  = data.pose.pose.orientation.x
		y  = data.pose.pose.orientation.y
		z = data.pose.pose.orientation.z
		w = data.pose.pose.orientation.w
		self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
###############################################################################


###################### Getting LIDAR data in a Python-Dictionary ################## 
	def laser_callback(self, msg):
		self.regions = {
		'front': min(min(msg.ranges[300:420]),100) 	,
		'right': min(min(msg.ranges[50:299]),100) ,
		'left' : min(min(msg.ranges[421:650]),100)
		}
################################################################################


#########################function for stopping values ##################
	def stopping_Vels(self):
		self.velocity_msg.linear.x=0.0
		self.velocity_msg.angular.z=0.0
		self.data_pub.publish(self.velocity_msg)
		
#######################################################################				


####################### function for Obstacle avoidance #######################
	def control_vel(self):
		self.rate.sleep()
		waypoints = self.Waypoints(self.resolCount)
		self.waypointsVisit(waypoints)
		
		while  True:
			x_left=self.x_targ-self.pose[0]
			y_left=self.y_targ-self.pose[1]
			theta_goal =  math.atan2(y_left,x_left)
			e_theta = (theta_goal-self.pose[2])*5
			distance = math.sqrt(math.pow(x_left,2)+math.pow(y_left,2))
			
			self.velocity_msg.linear.x = 1.0
			

			if(self.regions['front']<=1.9 or self.regions['right']<=0.6):
				
				self.velocity_msg.linear.x=0.1
				self.velocity_msg.angular.z = 3.0
				self.data_pub.publish(self.velocity_msg) 
			
			if(self.regions['front']>1.9 and (self.regions['right']>0.6 and self.regions['right']<0.9)):
				
				self.velocity_msg.angular.z = 0.0
				self.data_pub.publish(self.velocity_msg) 

			if(self.regions['front']>=1.9 and self.regions['right']>=0.9): 
			
				self.velocity_msg.angular.z = min(e_theta,2.0)
				self.data_pub.publish(self.velocity_msg) 
			
			if(distance<0.1):
				self.stopping_Vels()
				exit()
				
###############################################################################





if __name__ == '__main__':
    try:
    	control_loop = Controller()
    	control_loop.control_vel()
    except rospy.ROSInterruptException:
        pass