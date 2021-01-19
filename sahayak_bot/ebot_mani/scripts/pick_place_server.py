#! /usr/bin/env python

import rospy
import math
import moveit_commander
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PointStamped, PoseStamped
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from actionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.srv import GetModelState
from ebot_mani.srv import PickPlace, PickPlaceResponse
import tf2_ros
import tf2_geometry_msgs

class Ur5Moveit:
    #################### init function ######################
    def __init__(self):
        rospy.init_node('pick_place_server')
        rospy.Service('pick_place', PickPlace, self.pick_place_callback)
        self.arm_group = moveit_commander.MoveGroupCommander('ur5_arm_planning_group')
        self.gripper_group = moveit_commander.MoveGroupCommander('ur5_gripper_planning_group')
        self.action_client = SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
        self.action_client.wait_for_server()

        self.robot = moveit_commander.RobotCommander()
        self.eef_link = self.arm_group.get_end_effector_link()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo('>>> Ur5Moveit init done')
        # self.print_current_state(self.arm_group)

        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        target_pose = get_model_state('dropbox', '').pose
        target_pose.position.z += 0.5
        target_pose.orientation.x = -0.707
        target_pose.orientation.y = -0.707
        target_pose.orientation.z = 0
        target_pose.orientation.w = 0
        target_pose = pose_to_pose_stamped(target_pose, 'odom')
        self.tf_target_pose_stamped = self.transform_pose(target_pose, 'ebot_base')

        self.go_to_predefined_pose(self.arm_group, 'allZeros')

    #################### helper functions ###################
    def transform_pose(self, stamped_pose, target_frame):
        source_frame = stamped_pose.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                        rospy.Time(0),
                                                        rospy.Duration(1.0))
            return tf2_geometry_msgs.do_transform_pose(stamped_pose, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('transform error')
            return stamped_pose

    def attach_object(self, box_name):
        touch_links = self.robot.get_link_names(group='ur5_arm_planning_group')
        self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)

    def detach_object(self, box_name):
        self.scene.remove_attached_object(self.eef_link, name=box_name)
        self.scene.remove_world_object(box_name)

    ################### moveit functions ###################
    def go_to_predefined_pose(self, planning_group, arg_pose_name):
        planning_group.set_named_target(arg_pose_name)
        plan = planning_group.plan()
        goal = ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()

    def automata(self, object_pose, target_pose, width = 0.05):
        object_pose.position.z += 0.1
        if not self.go_to_pose(self.arm_group, object_pose, 'arm_group'):
            return False
        object_pose.position.z -= 0.1
        if not self.go_to_pose(self.arm_group, object_pose, 'arm_group'):
            return False
        # if not self.go_to_predefined_pose(self.gripper_group, 'gripperClose'):
        #     return False
        if not self.set_joint_angles(self.gripper_group, get_joint_angles(0.055), 'gripper_group'):
            return False
        object_pose.position.z += 0.1
        if not self.go_to_pose(self.arm_group, object_pose, 'arm_group'):
            return False
        # if not self.go_to_pose(self.arm_group, target_pose, 'arm_group'):
        #     return False
        if not self.go_to_predefined_pose(self.arm_group, 'dropPosition'):
            return False
        if not self.go_to_predefined_pose(self.gripper_group, 'gripperOpen'):
            return False
        if not self.go_to_predefined_pose(self.arm_group, 'allZeros'):
            return False
        return True

    def go_to_pose(self, planning_group, pose, planning_group_name=""):
        # self.print_current_state(planning_group, planning_group_name + ' Current')
        planning_group.set_pose_target(pose)
        flag_plan = planning_group.go(wait=True)
        # self.print_current_state(planning_group, planning_group_name + ' Final')

        if flag_plan:
            rospy.loginfo('>>> {} go_to_pose() success'.format(planning_group_name))
        else:
            rospy.loginfo('>>> {} go_to_pose() failed'.format(planning_group_name))
        return flag_plan

    def set_joint_angles(self, planning_group, joint_angles, planning_group_name=""):
        # self.print_current_state(planning_group, planning_group_name + ' Current')
        planning_group.set_joint_value_target(joint_angles)
        flag_plan = planning_group.go(wait=True)
        # self.print_current_state(planning_group, planning_group_name + ' Final')

        if flag_plan:
            rospy.loginfo('>>> set_joint_angles() success')
        else:
            rospy.loginfo('>>> set_joint_angles() failed')
        return flag_plan

    def print_current_state(self, planning_group, pose_prefix=""):
        rospy.loginfo('>>> {} Pose: {}'.format(pose_prefix, planning_group.get_current_pose().pose))
        rospy.loginfo('>>> {} joint angles: {}'.format(pose_prefix, planning_group.get_current_joint_values()))

    ################### destructor #########################
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('object ur5 deleted')

    ################## Service callback ####################
    def pick_place_callback(self, req):
        object_pose = req.objectPose.pose
        tf_object_pose_stamped = self.transform_pose(object_pose, 'ebot_base')
        tf_object_pose_stamped.pose.position.y -= END_EFF_OFFSET
        tf_object_pose_stamped.pose.orientation.x = -0.707059138329
        tf_object_pose_stamped.pose.orientation.y = -0.707152273758
        tf_object_pose_stamped.pose.orientation.z = 0.00121339752839
        tf_object_pose_stamped.pose.orientation.w = 0.00125071584406

        pose_stamped_to_point(tf_object_pose_stamped, '/tf_object_pose', (0, END_EFF_OFFSET, 0))
        pose_stamped_to_point(self.tf_target_pose_stamped, '/tf_target_pose')

        status = self.automata(tf_object_pose_stamped.pose, self.tf_target_pose_stamped.pose, width = 0.055)
        # if not self.go_to_pose(self.arm_group, self.target_pose, 'arm_group'):
        #     return False
        return PickPlaceResponse(status)
        # return True

def get_joint_angles(joint_distance):
    return [-0.80/0.085 * joint_distance + 0.80]

def pose_stamped_to_point(pose_stamped, pub_topic='', offset=(0,0,0)):
    point = PointStamped()
    point.header = pose_stamped.header
    point.point.x = pose_stamped.pose.position.x + offset[0]
    point.point.y = pose_stamped.pose.position.y + offset[1]
    point.point.z = pose_stamped.pose.position.z + offset[2]

    if pub_topic != '':
        pub = rospy.Publisher(pub_topic, PointStamped, queue_size=1)
        pub.publish(point)
    return point

def pose_to_pose_stamped(pose, frame_id):
    pose_stamped = PoseStamped()
    header = Header()
    header.seq = 0
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    pose_stamped.header = header
    pose_stamped.pose = pose
    return pose_stamped

if __name__ == '__main__':
    END_EFF_OFFSET = 0.132
    ur5 = Ur5Moveit()
    try:
        while not rospy.is_shutdown():
            pose_stamped_to_point(ur5.arm_group.get_current_pose(),
                                      pub_topic='/end_effector',
                                      offset=(0,END_EFF_OFFSET,0))
    except KeyboardInterrupt as e:
        print(e)
    del(ur5)
