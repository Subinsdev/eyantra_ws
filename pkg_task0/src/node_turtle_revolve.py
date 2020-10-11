#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleRotate:
    def __init__(self):
        rospy.init_node('turtle_rotate')
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.vel_msg = Twist()

    def pose_callback(self, data):
        if (data.theta > -0.01 and data.theta < 0):
            self.vel_msg = Twist()
            self.publish()
            rospy.loginfo("goal reached")
            rospy.signal_shutdown('goal_reached')

        self.rotate()
        self.publish()

    def rotate(self, radius = 1.5, linear_speed = 1.0):
        self.vel_msg.linear.x = linear_speed
        self.vel_msg.angular.z = (linear_speed * 1.0)/radius

    def publish(self):
        rospy.loginfo("Moving in a circle")
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        tr = TurtleRotate()
        if (not rospy.is_shutdown()):
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
