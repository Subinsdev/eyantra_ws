#!/usr/bin/env python
'''
Eyantra
'''
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

POSE = [0, 0, 0]
REGIONS = dict()
P = 0.8
K = 0.5
NUM_POINTS = 20

def waypoints(x_1):
    '''
    Generates Waypoints, depending upon NUM_POINTS
    params: t (float): x coordinte of ebot, POSE[0]
    return: [x, y] (list): next generated coordintes
    '''
    if x_1 > 2 * math.pi:
        x_2, y_2 = 12.5, 0
    else:
        mul = 2 * math.pi / NUM_POINTS
        x_2 = (x_1 // mul + 1) * mul
        y_2 = 2 * math.sin(x_2) * math.sin(x_2 / 2)
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
        theta_goal = math.atan((y_2 - y_1)/(x_2 - x_1))
        e_theta = ebot_theta - theta_goal

        state = 'traj'
        final_dist, laser_error = 0, 0
        if x_1 > 2 * math.pi:
            state = 'wall'
            laser_error = 1/REGIONS['fleft'] - 1/REGIONS['front'] - 1/REGIONS['fright']
            e_theta = e_theta + K * laser_error

            final_dist = math.sqrt((x_2 - x_1) ** 2 + (y_2 - y_1) ** 2)
            if final_dist < 0.5:
                break

        print("x_1: {0:f}, y_1: {1:f}, x_2: {2:f}, y_2: {3:f}, ebot: {4:f}, \
               gl: {5:f}, er: {6:f}".format(x_1, y_1, x_2, y_2, ebot_theta,
                                            theta_goal, e_theta))
        print "{0:s}, l_e: {1:f}, e_t: {2:f}, \
               f_d: {3:f}".format(state, laser_error, e_theta, final_dist)
        velocity_msg.linear.x = 0.1
        velocity_msg.angular.z = -P * e_theta
        pub.publish(velocity_msg)
        # print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

def odom_callback(data):
    '''
    Callback function for /odom topic
    params: msg (nav_msgs/Odometry): Odometry data from ebot
    return: None
    '''
    global POSE
    q_x = data.POSE.POSE.orientation.x
    q_y = data.POSE.POSE.orientation.y
    q_z = data.POSE.POSE.orientation.z
    q_w = data.POSE.POSE.orientation.w
    POSE = [data.POSE.POSE.position.x, data.POSE.POSE.position.y,
            euler_from_quaternion([q_x, q_y, q_z, q_w])[2]]

def laser_callback(msg):
    '''
    Callback function for /ebot/laser/scan topic
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
