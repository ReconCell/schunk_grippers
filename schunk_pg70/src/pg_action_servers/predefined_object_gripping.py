# Action server for using the Schunk PG70 gripper. The action server is activated
# by sending it a string that points it to a rosparam entry. The action server
# then reads the data from the parameter server and moves accordingly
#
# Author: Timotej Gaspar
# Email: timotej.gaspar@ijs.si

import rospy

from schunk_pg70 import GripNamedObject

class GrippingNamedObjects(object):
    def __init__(self):
        rospy.init_node('grip_named_object_action_server')
        rospy.loginfo("Hello!")