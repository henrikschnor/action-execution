#!/usr/bin/env python

from mcr_perception_msgs.msg import Object

from ae_ros_message_converter.converters.converter_base import ConverterBase
from ae_ros_message_converter.converters.pose_converter import PoseStampedConverter
from ae_ros_message_converter.converters.bbox_converter import BoundingBoxConverter

from action_execution.geometry.object import Object3d

class ObjectConverter(ConverterBase):
    def __init__(self):
        super(ObjectConverter, self).__init__()

    @staticmethod
    def convert_ros_msg(msg):
        '''Converts an "mcr_perception_msgs.msg.Object" message to an
        "action_execution.geometry.object.Object3d" object.

        Keyword argumens:
        msg -- an "mcr_perception_msgs.msg.Object" message

        '''
        obj = Object3d()

        obj.id = msg.name
        obj.type = msg.category
        obj.pose = PoseStampedConverter.convert_ros_msg(msg.pose)
        obj.bbox = BoundingBoxConverter.convert_ros_msg(msg.bounding_box)

        return obj

    @staticmethod
    def convert_to_ros_msg(obj):
        '''Converts an 'action_execution.geometry.object.Object3d' object to an
        'mcr_perception_msgs.msg.Object' message.

        Keyword argumens:
        obj -- an 'action_execution.geometry.object.Object3d' object

        '''
        obj_msg = Object()

        obj_msg.name = obj.id
        obj_msg.category = obj.type
        obj_msg.pose = PoseStampedConverter.convert_to_ros_msg(obj.pose)
        obj_msg.bounding_box = BoundingBoxConverter.convert_to_ros_msg(obj.bbox)

        return obj_msg
