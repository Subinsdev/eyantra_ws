#!/usr/bin/env python
'''
Ebot Navigation node, contains all algorithms to publish waypoints
to move_base action service which will generate global and local
maps and move the ebot across the map avoiding all obstacles.
'''
import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

def keypoints():
    '''
    Calls movebase_client for each waypoints and returns true when ebot
    completes all the waypoints successfully.
    params: None
    return: (bool): Status of the algorithm
    '''
    way_points = np.array([[0, 0], [-9.1, -1.2], [10.7, 10.5], [12.6, -1.5],
                           [18.2, -1.4], [-2, 4]])
    theta = np.arctan2(way_points[1:, 1] - way_points[:-1, 1],
                       way_points[1:, 0] - way_points[:-1, 0])

    # wp = np.array([[-9.1, -1.2], [10.7, 10.5], [12.8, -1.5], [18.2, -1.4], [-2, 4]])
    # theta = np.arctan2(wp[1:, 1] - wp[:-1, 1], wp[1:, 0] - wp[:-1, 0])
    # theta = np.append(theta, theta[-1])

    for pts, ang in zip(way_points[1:], theta):
        movebase_client(pts[0], pts[1], ang)
    return True

def movebase_client(wp_x, wp_y, theta):
    '''
    Pulishes target position to move_base server throught move_base/goal topic
    params: x (float): x coordinte of target position
            y (float): y coordinte of target position
            theta (float): required yaw of the robot
    '''
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()                     # Waits until the action server starts
    goal = MoveBaseGoal()                        # Creates a new goal
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp_x
    goal.target_pose.pose.position.y = wp_y

    quart = quaternion_from_euler(0.0, 0.0, theta)
    goal.target_pose.pose.orientation = Quaternion(*quart)

    client.send_goal(goal)                      # Sends the goal to the action server.
    wait = client.wait_for_result()             # Waits for the server to finishes.
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    return client.get_result()                  # Result of executing the action

if __name__ == '__main__':
    try:
        rospy.init_node("task2_node")
        RESULT = keypoints()
        if RESULT:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.loginfo("Goal execution failed!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
