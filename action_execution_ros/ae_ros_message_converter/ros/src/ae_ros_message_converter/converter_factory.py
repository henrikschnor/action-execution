#!/usr/bin/env python

import rospy

from ae_ros_message_converter.converters.pose_converter import PoseStampedConverter
from ae_ros_message_converter.converters.bbox_converter import BoundingBoxConverter
from ae_ros_message_converter.converters.object_converter import ObjectConverter

class ConverterFactory(object):
    def __init__(self):
        pass

    @staticmethod
    def convert_ros_msg(msg):
        '''Converts 'msg' to an appropriate type if it is a known message;
        returns None otherwise.

        Keyword:
        msg -- a ros message

        '''
        if type(msg).__name__.lower() == 'posestamped':
            return PoseStampedConverter.convert_ros_msg(msg)
        elif type(msg).__name__.lower() == 'boundingbox':
            return BoundingBoxConverter.convert_ros_msg(msg)
        elif type(msg).__name__.lower() == 'object':
            return ObjectConverter.convert_ros_msg(msg)

        rospy.loginfo('[ConverterFactory] Unknown message type; ignoring request')
        return None

    @staticmethod
    def convert_to_ros_msg(obj):
        '''Converts 'obj' to an appropriate message if it is a known type;
        returns None otherwise.

        Keyword:
        msg -- a ros message

        '''
        if type(obj).__name__.lower() == 'pose3':
            return PoseStampedConverter.convert_to_ros_msg(obj)
        elif type(obj).__name__.lower() == 'bbox3':
            return BoundingBoxConverter.convert_to_ros_msg(obj)
        elif type(obj).__name__.lower() == 'object3d':
            return ObjectConverter.convert_to_ros_msg(obj)

        rospy.loginfo('[ConverterFactory] Unknown type; ignoring request')
        return None
