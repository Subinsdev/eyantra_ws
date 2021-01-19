import math
import random
import rospy
import rospkg
import tf

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel

from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

def initial_setup():
    """ Turns of gravity so that model do not fall
        Deletes ground plane

        Params: None
        Return: None
    """
    # Wait for services
    rospy.wait_for_service('gazebo/get_model_state')
    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/get_physics_properties')
    rospy.wait_for_service('gazebo/set_physics_properties')
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    # get physics_properties
    get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    physics_properties = get_physics_properties_prox()

    # gravity is changed to 0
    physics_properties.gravity.z = 0

    # set the modified physics_properties
    set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
    set_physics_properties_prox(physics_properties.time_step,
                                physics_properties.max_update_rate,
                                physics_properties.gravity,
                                physics_properties.ode_config)

    # delete ground_plane
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox('ground_plane')

def spawn_model(model_name, pkg_name='ebot_gazebo'):
    """ Spawns model infront of depth camera
        Params: model_name, pkg_name
        Return: None
    """
    initial_pose = Pose()
    initial_pose.position.x = 7.971280
    initial_pose.position.y = 3.239280
    initial_pose.position.z = 0.869991

    model_path = rospkg.RosPack().get_path(pkg_name)+'/models/'
    model_xml = ''

    with open(model_path + model_name + '/model.sdf', 'r') as xml_file:
        model_xml = xml_file.read().replace('\n', '')

    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox('training_model', model_xml, '', initial_pose, 'world')

def delete_model():
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox('training_model')

def capture_sample(topic='/sensor_stick/point_cloud'):
    """ Captures a PointCloud2 using RGBD camera
        Params: topic
        Return: PointCloud2
    """
    get_model_state_prox = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    model_state = get_model_state_prox('training_model', 'world')

    set_model_state_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    # roll = random.uniform(0, 2*math.pi)
    # pitch = random.uniform(0, 2*math.pi)
    roll = 0
    pitch = 0
    yaw = random.uniform(0, 2*math.pi)

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    model_state.pose.orientation.x = quaternion[0]
    model_state.pose.orientation.y = quaternion[1]
    model_state.pose.orientation.z = quaternion[2]
    model_state.pose.orientation.w = quaternion[3]

    sms_req = SetModelStateRequest()
    sms_req.model_state.pose = model_state.pose
    sms_req.model_state.twist = model_state.twist
    sms_req.model_state.model_name = 'training_model'
    sms_req.model_state.reference_frame = 'world'
    set_model_state_prox(sms_req)

    return rospy.wait_for_message(topic, PointCloud2)
