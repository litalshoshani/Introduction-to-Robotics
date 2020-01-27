#!/usr/bin/python
#
# wander_bot.py
#
# Author: Lotan & Lital
#

import sys
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi

'''
    the wander bot logic
'''
class Stopper(object):

    '''
        robot initialization
    '''
    def __init__(self, forward_speed, rotation_speed, min_angle, max_angle):
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        self.min_angle = min_angle
        self.max_angle = max_angle

        self.leftmost_angle = -90 / 360 * math.pi
        self.rightmost_angle = 90 / 360 * math.pi
        self.min_dist_from_obstacle = 1
        self.keep_moving = True
        self.angle_increment = 3.14 / 100

        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

    '''
        moving the robot forward
    '''
    def move_forward(self,DistanceToGO):
        move_msg = Twist()
        move_msg.linear.x = self.forward_speed

        # taking current time
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        rate = rospy.Rate(10)

        # running on x axis until we cover all the distance
        while (abs(DistanceToGO - current_distance) > 0.001):
            self.command_pub.publish(move_msg)
            rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_distance = self.forward_speed * (t1 - t0)

        # setting the motion back to zero to stop moving
        move_msg.linear.x = 0
        self.command_pub.publish(move_msg)

    '''
        rotating the bot
    '''
    def rotateTillDesiredAngleReached(self,current_angle,angleDesired):
        RotationSpeed = self.rotation_speed
        RotationAngle = 0

        deg_curr = current_angle/math.pi * 180
        deg_Desired = angleDesired/math.pi * 180

        if (deg_curr < 0):
            deg_curr = 360+deg_curr
        if (deg_Desired < 0):
            deg_Desired = 360 + deg_Desired

        if (abs(deg_curr-deg_Desired) <= 10):
            RotationAngle = 0
        elif (angleDesired > current_angle):
            RotationAngle = angleDesired - current_angle
        else:
            RotationAngle = 2*math.pi - current_angle + angleDesired

        # Drive forward at a given speed.  The robot points up the x-axis.
        # The default constructor will set all commands to 0
        msg = Twist()

        # taking time again
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        msg.angular.z = RotationSpeed

        if (RotationAngle < 0):
            rate = rospy.Rate(10)
            # turning around until we get the desired angle
            while (abs(current_angle) - abs(RotationAngle) <= 0):

                self.command_pub.publish(msg)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
                current_angle = -1*RotationSpeed * (t1 - t0)


        else:
            rate = rospy.Rate(10)
            # turning around until we get the desired angle
            while (current_angle - RotationAngle <= 0):

                self.command_pub.publish(msg)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
                current_angle = RotationSpeed * (t1 - t0)

        # setting motion back to zero to stop turning
        msg.angular.z = 0
        self.command_pub.publish(msg)

    '''
        helpful function - returns an array with relevant range
    '''
    def getValsInRange(self,ranges):
        myArr = []
        tmpAngle = 0
        for i in range(len(ranges)):
            tmpAngle = self.leftmost_angle + i*self.angle_increment
            if (tmpAngle > self.min_angle and tmpAngle < self.max_angle):
                myArr.append(ranges[i])
        return myArr

    '''
        getting distance and angle desired for next move
    '''
    def getDistAndAngleFromPoints(self,startP,endP):
        if (int(startP.x) == int(endP.x)):
            val = endP.y - startP.y
            if (val > 0.0):
                return abs(val),90.0/360 * 2*math.pi
            return abs(val),270.0/360 * 2*math.pi
        else:
            val = endP.x - startP.x
            if (val > 0.0):
                return abs(val),0.0
            return abs(val),180.0/360 * 2*math.pi

