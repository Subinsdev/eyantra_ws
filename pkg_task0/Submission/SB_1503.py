#!/usr/bin/env python
"""
This code uses turtlesim_node and implements rotation maneuver
"""
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleRotate(object):
    """
    TurtleRotate class consisting all required methods and variables for rotation
    """
    def __init__(self):
        rospy.init_node('turtle_rotate')
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.vel_msg = Twist()

    def pose_callback(self, data):
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

    def rotate(self, radius=1.5, linear_speed=1.0):
        """
        Calculates the linear and angular speed to be published
        parameters:
            radius (float, default=1.5): radius of the circle.
            linear_speed (float, default=1.0): forward speed of the bot
        """
        self.vel_msg.linear.x = linear_speed
        self.vel_msg.angular.z = (linear_speed * 1.0)/radius

    def publish(self):
        """
        Publishes the vel_msg to the /turtle1/cmd_vel topic
        """
        rospy.loginfo("Moving in a circle")
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        TR = TurtleRotate()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as exception:
        print exception
