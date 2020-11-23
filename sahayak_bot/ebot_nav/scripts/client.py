#!/usr/bin/env python
'''
Ebot controller node, contains all algorithms to follow a sinosodial trajectory
with Odometry, avoid obstacles using LaserScan, and publish Proportional control
command to skid steer drive to control the robot.
'''
import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

def keypoints():
    wp = np.array([[0,0], [-9.1, -1.2], [10.7, 10.5], [12.6, -1.6], [18.2, -1.4], [-2, 4]])
    # theta = np.arctan2(wp[1:, 1] - wp[:-1, 1], wp[1:, 0] - wp[:-1, 0])
    # theta = np.append(theta, theta[-1])
    theta = np.arctan2(wp[1:, 1] - wp[:-1, 1], wp[1:, 0] - wp[:-1, 0])
    for i, (pts, ang) in enumerate(zip(wp[1:], theta)):
        print(i, pts, ang)
        movebase_client(pts[0], pts[1], ang)

def movebase_client(x, y, theta):
    '''
    Pulishes target position to move_base server throught move_base/goal topic
    params: x (float): x coordinte of target position
            y (float): y coordinte of target position
            theta (float): required yaw of the robot
    '''
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()                     # Waits until the action server starts
    goal = MoveBaseGoal()                        # Creates a new goal
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0.0, 0.0, np.radians(theta))
    goal.target_pose.pose.orientation = Quaternion(*q)

    client.send_goal(goal)                      # Sends the goal to the action server.
    wait = client.wait_for_result()             # Waits for the server to finishes.
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()              # Result of executing the action

if __name__ == '__main__':
    '''
    If the python node is executed as main process
    Initializes a rospy node to let the SimpleActionClient publish and subscribe
    '''
    try:
        rospy.init_node('task2_node')
        keypoints()
        # result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
