#!/usr/bin/env python
'''
Ebot controller node, contains all algorithms to follow a sinosodial trajectory
with Odometry, avoid obstacles using LaserScan, and publish Proportional control
command to skid steer drive to control the robot.
'''
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

POSE = [0, 0, 0]            # List to contain Odometry data
REGIONS = dict()            # Dict to contain LaserScan data
P = 1                       # Proportionality constant
K = 2                       # Obstacle avoidance constant
NUM_POINTS = 20             # Number of keypoints between 0, 2 PI

def waypoints(x_1):
    '''
    Generates Waypoints, depending upon NUM_POINTS
    params: t (float): x coordinte of ebot, POSE[0]
    return: [x, y] (list): next generated coordintes
    '''
    if x_1 > 2 * math.pi:                           # If bot crossed 2PI
        x_2, y_2 = 12.5, 0                          # then goal is (12.0, 0)
    else:
        mul = 2 * math.pi / NUM_POINTS              # Factor 2PI/NUM_POINTS
        x_2 = (x_1 // mul + 1) * mul                # generate a point ahead
        y_2 = 2 * math.sin(x_2) * math.sin(x_2 / 2) # y_coord from equation
    return [x_2, y_2]

def control_loop():
    '''
    Main control loop, contains obstacle avoidance,
    trajectory following algorithm and P controller
    '''
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():
        x_1, y_1, ebot_theta = POSE
        x_2, y_2 = waypoints(x_1)

        # Go to goal controller
        theta_goal = math.atan2(y_2 - y_1, x_2 - x_1)
        e_theta = theta_goal - ebot_theta
        balance = P * e_theta

        if x_1 > 2 * math.pi:
            # For obstacle avoidance, a non-linear approach is used
            laser_error = (1/(REGIONS['fright']) - 1/(REGIONS['fleft']) +
                           2 * 1/(REGIONS['front']) - 1/(REGIONS['bright'])/1.5
                           + 1/(REGIONS['bleft'])/1.5)

            # The above constants are experimentally determined
            # A weighted sum of goal error and obstacle error
            balance = P * e_theta + K * (laser_error)

            # Check if goal is reached, based on euclidean distance
            final_dist = math.sqrt((x_2 - x_1) ** 2 + (y_2 - y_1) ** 2)
            if final_dist < 0.1:
                break

        velocity_msg.linear.x = 0.2
        velocity_msg.angular.z = balance
        pub.publish(velocity_msg)
        print "Controller message pushed at {}".format(rospy.get_time())
        rate.sleep()

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

def odom_callback(data):
    '''
    Callback function for /odom topic, assign Odometry data like x and y
    coori and euler angle to POSE variable.
    params: msg (nav_msgs/Odometry): Odometry data from ebot
    return: None
    '''
    global POSE
    q_x = data.pose.pose.orientation.x
    q_y = data.pose.pose.orientation.y
    q_z = data.pose.pose.orientation.z
    q_w = data.pose.pose.orientation.w
    POSE = [data.pose.pose.position.x, data.pose.pose.position.y,
            euler_from_quaternion([q_x, q_y, q_z, q_w])[2]]

def laser_callback(msg):
    '''
    Callback function for /ebot/laser/scan topic, the function calculates min
    distance in a specific range and assign it to global variable REGIONS.
    params: msg (sensor_msgs/LaserScan): laser sensor data and ranges
    return: None
    '''
    global REGIONS
    REGIONS = {
        'bright': min(msg.ranges[0:720/5]),
        'fright': min(msg.ranges[720/5:720*2/5]),
        'front': min(msg.ranges[720*2/5:720*3/5]),
        'fleft': min(msg.ranges[720*3/5:720*4/5]),
        'bleft': min(msg.ranges[720*4/5:720]),
    }

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
