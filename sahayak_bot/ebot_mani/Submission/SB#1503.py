#! /usr/bin/env python

import rospy
import math
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from actionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler

class Ur5Moveit:
    def __init__(self):
        rospy.init_node('ur5_moveit')
        self.arm_group = moveit_commander.MoveGroupCommander('ur5_arm_planning_group')
        self.gripper_group = moveit_commander.MoveGroupCommander('ur5_gripper_planning_group')
        self.action_client = SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
        self.action_client.wait_for_server()

        self.robot = moveit_commander.RobotCommander()
        self.eef_link = self.arm_group.get_end_effector_link()
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.loginfo('>>> Ur5Moveit init done')

    def get_joint_angles(self, joint_distance):
        return [-0.80/0.085 * joint_distance + 0.80]

    def get_item_pose(self, (xyz, rpy)):
        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        quaternion = quaternion_from_euler(*rpy)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def attach_object(self, box_name):
        touch_links = self.robot.get_link_names(group='ur5_arm_planning_group')
        self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)

    def detach_object(self, box_name):
        self.scene.remove_attached_object(self.eef_link, name=box_name)
        self.scene.remove_world_object(box_name)

    def pick_place(self, item):
        pre_grasp_pose = list(item['item_pose'][0])
        pre_grasp_pose[2] += 0.08
        pre_grasp_pose = (pre_grasp_pose, item['item_pose'][1])

        if not self.go_to_pose(self.arm_group, self.get_item_pose(pre_grasp_pose), 'arm_group'):
            return False
        if not self.go_to_pose(self.arm_group, self.get_item_pose(item['item_pose']), 'arm_group'):
            return False
        if not self.set_joint_angles(self.gripper_group, self.get_joint_angles(item['width']), 'gripper_group'):
            return False
        if not self.go_to_pose(self.arm_group, self.get_item_pose(item['target_pose']), 'arm_group'):
            return False
        if not self.set_joint_angles(self.gripper_group, [0], 'gripper_group'):
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

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('object ur5 deleted')

if __name__ == '__main__':
    ur5 = Ur5Moveit()
    try:
        item_list = {'object_1': {'item_pose': ([0.56, -0.02, 0.84], [-0.707,1.57,0]),
                                  'target_pose': ([0, 0.71, 1], [0.707,1.57,0]),
                                  'width': 0.055},
                     'object_2': {'item_pose': ([0.46, 0.22, 0.84], [1,1.57,0]),
                                  'target_pose': ([0, -0.71, 1], [0.707,1.57,0]),
                                  'width': 0.025},
                     'object_3': {'item_pose': ([0.56, -0.24, 0.89], [1.8,1.57,0]),
                                  'target_pose': ([0.2, -0.71, 1], [0.707,1.57,0]),
                                  'width': 0.053}
                     }
        for item in item_list:
            if not ur5.pick_place(item_list[item]):
                rospy.loginfo('>>> Task Failed for {}'.format(item))
                break

    except KeyboardInterrupt as e:
        print(e)
    del(ur5)
