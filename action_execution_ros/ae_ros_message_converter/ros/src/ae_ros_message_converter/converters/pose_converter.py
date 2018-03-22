#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped

from ae_ros_message_converter.converters.converter_base import ConverterBase

from action_execution.geometry.pose import Pose3
from action_execution.extern.transformations import euler_from_quaternion, quaternion_from_euler

class PoseStampedConverter(ConverterBase):
    def __init__(self):
        super(PoseStampedConverter, self).__init__()

    @staticmethod
    def convert_ros_msg(msg):
        '''Converts a 'geometry_msgs.msg.PoseStamped' message to an
        'action_execution.geometry.pose.Pose3' object.

        Keyword argumens:
        msg -- a 'geometry_msgs.PoseStamped' message

        '''
        pose = Pose3()

        pose.frame_id = msg.header.frame_id

        pose.position.x = msg.pose.position.x
        pose.position.y = msg.pose.position.y
        pose.position.z = msg.pose.position.z

        euler_rot = euler_from_quaternion([msg.pose.orientation.x,
                                           msg.pose.orientation.y,
                                           msg.pose.orientation.y,
                                           msg.pose.orientation.w])

        pose.orientation.x = euler_rot[0]
        pose.orientation.y = euler_rot[1]
        pose.orientation.z = euler_rot[2]

        return pose

    @staticmethod
    def convert_to_ros_msg(obj):
        '''Converts an 'action_execution.geometry.pose.Pose3' object to a
        '"'geometry_msgs.msg.PoseStamped' message.

        Keyword argumens:
        obj -- an 'action_execution.geometry.pose.Pose3' object

        '''
        pose = PoseStamped()
        pose.header.frame_id = obj.frame_id

        pose.pose.position.x = obj.position.x
        pose.pose.position.y = obj.position.y
        pose.pose.position.z = obj.position.z

        quat_orientation = quaternion_from_euler(obj.orientation.x,
                                                 obj.orientation.y,
                                                 obj.orientation.z)

        pose.pose.orientation.x = quat_orientation[0]
        pose.pose.orientation.y = quat_orientation[1]
        pose.pose.orientation.z = quat_orientation[2]
        pose.pose.orientation.w = quat_orientation[3]

        return pose
