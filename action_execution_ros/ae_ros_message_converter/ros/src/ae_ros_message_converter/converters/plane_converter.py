#!/usr/bin/env python

from mcr_perception_msgs.msg import Plane as MCRPlane

from ae_ros_message_converter.converters.converter_base import ConverterBase
from ae_ros_message_converter.converters.pose_converter import PoseStampedConverter
from ae_ros_message_converter.converters.bbox_converter import BoundingBoxConverter

from action_execution.geometry.object import Plane

class PlaneConverter(ConverterBase):
    def __init__(self):
        super(PlaneConverter, self).__init__()

    @staticmethod
    def convert_ros_msg(msg):
        '''Converts an "mcr_perception_msgs.msg.Plane" message to an
        "action_execution.geometry.plane.Plane" object.

        Keyword argumens:
        msg -- an "mcr_perception_msgs.msg.Plane" message

        '''
        obj = Plane()

        obj.id = msg.name

        obj.pose.position.x = msg.plane_point.x
        obj.pose.position.y = msg.plane_point.y
        obj.pose.position.z = msg.plane_point.z

        obj.bbox.min.x = msg.limits.min_x
        obj.bbox.min.y = msg.limits.min_y
        obj.bbox.max.x = msg.limits.max_x
        obj.bbox.max.y = msg.limits.max_y

        return obj

    @staticmethod
    def convert_to_ros_msg(obj):
        '''Converts an 'action_execution.geometry.plane.Plane' object to an
        'mcr_perception_msgs.msg.Plane' message.

        Keyword argumens:
        obj -- an 'action_execution.geometry.plane.Plane' object

        '''
        obj_msg = MCRPlane()

        obj_msg.name = obj.id

        obj_msg.plane_point.x = obj.pose.position.x
        obj_msg.plane_point.y = obj.pose.position.y
        obj_msg.plane_point.z = obj.pose.position.z

        obj_msg.limits.min_x = obj.bbox.min.x
        obj_msg.limits.max_x = obj.bbox.min.y
        obj_msg.limits.max_x = obj.bbox.max.x
        obj_msg.limits.max_x = obj.bbox.max.y

        return obj_msg
