# Action server for using the Schunk PG70 gripper. The action server is activated
# by sending it a string that points it to a rosparam entry. The action server
# then reads the data from the parameter server and moves accordingly
#
# Author: Timotej Gaspar
# Email: timotej.gaspar@ijs.si

import rospy
import actionlib

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from schunk_pg70.msg import GripNamedObjectAction, GripNamedObjectFeedback, GripNamedObjectResult, GripNamedObjectGoal
from schunk_pg70.srv import set_position


MAX_SPEED = 82
MAX_ACCEL = 300
DEFAULT_SPEED = MAX_SPEED*0.2
DEFAULT_ACCEL = MAX_ACCEL*0.1

GRIPPER_OPEN = 65

POS_TOLERANCE = 0.2

M_TO_MM = 1000

SERVICE_TIMEOUT = 3


class GrippingNamedObjects(object):
    def __init__(self):
        try:
            # Create a node
            rospy.init_node('grip_named_object_action_server')

            # Create ROS service proxies
            self.set_grip_pos = rospy.ServiceProxy('set_position', set_position)
            self.grip_stop = rospy.ServiceProxy('stop', set_position)

            # Subscribe to the gripper state
            rospy.topics.Subscriber('joint_states', JointState, self.joint_state_sub)

            # Provide a way to open the gripper
            self._open_grip_srv = rospy.Service('open_gripper', Empty, self.open_gripper_cb)

            # Create the action server
            self._as = actionlib.SimpleActionServer('grip_named_object', GripNamedObjectAction, self.execute_cb)
            rospy.loginfo("Started 'grip_named_object_action_server'")

            self._rate = rospy.Rate(20)

            rospy.spin()
        except Exception as e:
            rospy.logerr('Exception in initialization:\n{0}'.format(e))

    def joint_state_sub(self, data):
        self.gripper_joints = data.position
        self.gripper_gap = (self.gripper_joints[0] + self.gripper_joints[1])*M_TO_MM

    def open_gripper_cb(self, req):
        self.set_grip_pos(GRIPPER_OPEN, MAX_SPEED, MAX_ACCEL)

        time_start = rospy.get_time()
        while (abs(GRIPPER_OPEN - self.gripper_gap) > POS_TOLERANCE and
                (rospy.get_time() - time_start) < SERVICE_TIMEOUT):
            self._rate.sleep()

        return []

    def execute_cb(self, goal):
        object_name = goal.object_name
        if object_name[0] is not '/':
            object_name = '/' + object_name

        # Get the info from the parameter server
        try:
            grasping_gap = rospy.get_param(object_name)
        except:
            rospy.logerr('Entered object [{0}] not found in parameter server'.format(object_name))
            self._as.set_aborted(GripNamedObjectResult(False))
            return


        # Set the gripper position
        self.set_grip_pos(grasping_gap, DEFAULT_SPEED, DEFAULT_ACCEL)

        # Wait for the position to be reached
        feedback = GripNamedObjectFeedback
        while abs(grasping_gap - self.gripper_gap) > POS_TOLERANCE:
            # feedback.gripper_position = self.gripper_gap
            self._as.publish_feedback(GripNamedObjectFeedback(
                abs(grasping_gap - self.gripper_gap)))

            if self._as.preempt_request:
                self._as.set_preempted(GripNamedObjectResult(False))
                return

        self._as.set_succeeded(GripNamedObjectResult(True))