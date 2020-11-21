#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

def keypoints(target):
    waypoints = [[-9.1, -1.2], [10.7, 10.5], [12.6, -1.9], [18.2, -1.4], [-2, 4]]
    return waypoints[target]
    q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))

def movebase_client():
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame
    goal.target_pose.pose.position.x = 12.6
    goal.target_pose.pose.position.y = -1.9

   # No rotation of the mobile base frame w.r.t. map frame
    q = quaternion_from_euler(0.0, 0.0, 0)
    goal.target_pose.pose.orientation = Quaternion(*q)

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('task2')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
