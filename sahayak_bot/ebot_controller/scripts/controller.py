#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import math

class EbotController:
    """
    EbotController class consisting all required methods and variables for the controller
    """
    def __init__(self):
        rospy.init_node('/ebot_controller')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.scan_callback)
        self.vel_msg = Twist()

    def odom_callback(self, data):
        """
        Callback function is invoked when new messages are received.
        parameters:
            data (turtlesim/Pose): received message from the topic /turtle1/pose
        """
        if (data.theta > -0.01 and data.theta < 0):
            self.vel_msg = Twist()
            self.publish()
            rospy.loginfo("goal reached")
            rospy.signal_shutdown('goal_reached')

        self.rotate()
        self.publish()

    def odom_callback(data):
        global pose
        x  = data.pose.pose.orientation.x;
        y  = data.pose.pose.orientation.y;
        z = data.pose.pose.orientation.z;
        w = data.pose.pose.orientation.w;
        pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

    def laser_callback(msg):
        global regions
        regions = {
            'bright':  	,
            'fright': 	,
            'front':  	,
            'fleft':  	,
            'bleft':   	,
        }


    def Waypoints(t):
        x  =
        y  =
        return [x,y]

    def publish(self):
        """
        Publishes the vel_msg to the /cmd_vel topic
        """
        rospy.loginfo("Moving in a circle")
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        ebot = EbotController()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as exception:
        print exception
