#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
import math

Destination = 0
obstacle =0
distance = 0
ebot_theta = 0
pose = [0,0,0]
P=0.5
pub=0

def odom_callback(data):
    global pose , goal, goal_status , ebot_theta  ,distance
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [round(data.pose.pose.position.x,2),round(data.pose.pose.position.y,2),round(euler_from_quaternion([x,y,z,w])[2],2)]
    


def laser_callback(path):
    global regions , obstacle
    regions = {
        'right':  min(min(path.ranges[70:299]), 5),    
        'front':  min(min(path.ranges[300:480]), 5),
        'left':   min(min(path.ranges[491:]), 5),
    }
    if(regions['front']<1):
        obstacle = 1


def Waypoints(t):
    
        x  =  t+0.6
        y  = round(2*math.sin(x)*math.sin(x/2),2)
        if(x >= 6.4 ):
            
            return [12.5,0]
        return [x,y]


def pass_obstacle():
    global regions , obstacle
    msg= Twist()
    while(obstacle ==1 ):
        if(regions['front'] < 1.5 ):
            msg.linear.x = 0.6
            msg.angular.z = 5
            print("obst")
        elif(regions['front']>1 and regions['right']>0.5 and regions['right']<1 ):
            msg.angular.z = 0
            print("obst11")
        #elif(regions['front'] <1):
         #   msg.linear.x = 0
        elif(regions['front'] >2 and regions['right']>2 and regions['left'] >2 ) :
            obstacle =0
        
        pub.publish(msg)
        

    
def control_loop():

    global distance , ebot_theta ,pose , pub, Destination
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    goal = [0,0]
    while True: # not rospy.is_shutdown():
        
        #
        # Your algorithm to complete the obstacle course
        #
        if(obstacle == 1):
            pass_obstacle()
        distance_x = goal[0]-pose[0]
        distance_y = goal[1]-pose[1]
        distance = math.sqrt(math.pow(distance_x,2)+math.pow(distance_y,2))
        print(distance)
        if(distance<0.1):#distance_x <=0.1 and distance_y <=0.1):
            if (Destination ==1):
                print("reached")
                exit()
            goal = Waypoints(pose[0])

        theta_goal = atan2((goal[1]-pose[1]),(goal[0]-pose[0]))
        ebot_theta = theta_goal - pose[2] 
        print(pose,goal)
        #print(distance,goal_status)

        velocity_msg.linear.x = 0.8#P*distance
        if(ebot_theta > 0.1 ):
            velocity_msg.angular.z = 2#P*ebot_theta
            velocity_msg.linear.x = 0
        elif(ebot_theta < -0.1):
            velocity_msg.angular.z = -2#-P*ebot_theta
            velocity_msg.linear.x = 0
        else:
            velocity_msg.angular.z = 0
        print(ebot_theta,velocity_msg.angular.z)
        pub.publish(velocity_msg)
        #print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass