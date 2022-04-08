#!/usr/bin/env python3

import rospy
import actionlib
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time

class BalancingRobots():

    def __init__(self):

        self.VEL_MAX = 0.15
        self.ANG_MAX = math.pi/18

        self.myPose = [0.0, 0.0, 0.0]
        self.leftPose = [0.0, 0.0, 0.0]
        self.rightPose = [0.0, 0.0, 0.0]

        self.myVel = 0.0
        self.leftVel = 0.0
        self.rightVel = 0.0

        self.myOmega = 0.0

        self.rate = rospy.Rate(30)

        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        rospy.Subscriber('/left_odom', Odometry, self.callback_left_odom)
        rospy.Subscriber('/right_odom', Odometry, self.callback_right_odom)

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10) 

        self.k = 1
        self.eps = 0.0001

        self.startTime = time.time()
        self.timeKeep = []
        self.waypoints = []



    def velocity_convert(self, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 1 #modify if necessary
        
        ang = math.atan2(vel_y, vel_x)
        if ang < 0:
            ang += 2 * math.pi
        
        ang_err = min(max(ang - theta, - self.ANG_MAX), self.ANG_MAX)

        v_lin =  min(max(vel_x, - self.VEL_MAX), self.VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang


    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.myPose[0] = data.pose.pose.position.x
        self.myPose[1] = data.pose.pose.position.y

        self.myVel = data.twist.twist.linear.x
        self.myOmega = data.twist.twist.angular.z

        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        _, _, self.myPose[2] = euler_from_quaternion(quaternion)

        self.timeKeep.append(time.time() - self.startTime)
        self.waypoints.append((self.myPose[0], self.myPose[1]))


    def callback_left_odom(self, data):
        '''
        Get left robot data
        '''
        self.leftPose[0] = data.pose.pose.position.x
        self.leftPose[1] = data.pose.pose.position.y

        self.leftVel = data.twist.twist.linear.x

        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        _, _, self.leftPose[2] = euler_from_quaternion(quaternion)


    def callback_right_odom(self, data):
        '''
        Get right robot data
        '''
        self.rightPose[0] = data.pose.pose.position.x
        self.rightPose[1] = data.pose.pose.position.y

        self.rightVel = data.twist.twist.linear.x

        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        _, _, self.rightPose[2] = euler_from_quaternion(quaternion)


    def balancing(self):
        print("Hey, we're balancing ")
        cnt = 0
        while not rospy.is_shutdown():
            vel = [0, 0]
            vel[0] = self.k*(self.rightPose[0] - 2*self.myPose[0] + self.leftPose[0])
            v_lin, v_ang = self.velocity_convert(self.myPose[2], vel[0], vel[1])
            # rospy.loginfo(v_ang)

            vel_msg = Twist()
            vel_msg.linear.x = v_lin
            vel_msg.angular.z = 0
            self.pub_vel.publish(vel_msg)
            print(v_lin, vel[0])
            self.rate.sleep()
            
            if (abs(self.myVel) < self.eps and abs(self.rightVel) < self.eps and abs(self.leftVel) < self.eps) and cnt>1000:
                print("F")
                rospy.signal_shutdown("Balanced")
                break

            cnt += 1

        return self.timeKeep, self.waypoints
        

def plotData(timeKeep, waypoints):
    wpX = [x[0] for x in waypoints]
    wpY = [x[1] for x in waypoints]

    plt.plot(timeKeep, wpX)
    plt.xlabel("Time")
    plt.ylabel("X-coordinate")
    plt.title("X vs T")
    plt.grid()
    plt.show()

    plt.plot(wpX, wpY)
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.title("Robot Path")
    plt.grid()
    plt.show()


rospy.init_node('balancing', anonymous = True, disable_signals = True)

balancer = BalancingRobots()
timeKeep, waypoints = balancer.balancing()
plotData(timeKeep, waypoints)





