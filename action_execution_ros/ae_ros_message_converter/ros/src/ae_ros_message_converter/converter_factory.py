#!/usr/bin/env python

import rospy

from ae_ros_message_converter.converters.pose_converter import PoseStampedConverter
from ae_ros_message_converter.converters.bbox_converter import BoundingBoxConverter
from ae_ros_message_converter.converters.object_converter import ObjectConverter
from ae_ros_message_converter.converters.plane_converter import PlaneConverter

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
            print('[ae_convert_ros_msg] Converting a "geometry_msgs.msg.PoseStamped" object')
            return PoseStampedConverter.convert_ros_msg(msg)
        elif type(msg).__name__.lower() == 'boundingbox':
            print('[ae_convert_ros_msg] Converting an "mcr_perception_msgs.msg.BoundingBox" object')
            return BoundingBoxConverter.convert_ros_msg(msg)
        elif type(msg).__name__.lower() == 'object':
            print('[ae_convert_ros_msg] Converting an "mcr_perception_msgs.msg.Object" object')
            return ObjectConverter.convert_ros_msg(msg)
        elif type(msg).__name__.lower() == 'plane':
            print('[ae_convert_ros_msg] Converting an "mcr_perception_msgs.msg.Plane" object')
            return PlaneConverter.convert_ros_msg(msg)

        rospy.loginfo('[ae_convert_ros_msg] Unknown message type; ignoring request')
        return None

    @staticmethod
    def convert_to_ros_msg(obj):
        '''Converts 'obj' to an appropriate message if it is a known type;
        returns None otherwise.

        Keyword:
        obj -- an action execution type

        '''
        if type(obj).__name__.lower() == 'pose3':
            print('[ae_convert_to_ros_msg] Converting a "Pose3" object')
            return PoseStampedConverter.convert_to_ros_msg(obj)
        elif type(obj).__name__.lower() == 'bbox3':
            print('[ae_convert_to_ros_msg] Converting a "BBox3" object')
            return BoundingBoxConverter.convert_to_ros_msg(obj)
        elif type(obj).__name__.lower() == 'object3d':
            print('[ae_convert_to_ros_msg] Converting a "Object3d" object')
            return ObjectConverter.convert_to_ros_msg(obj)
        elif type(obj).__name__.lower() == 'plane':
            print('[ae_convert_to_ros_msg] Converting a "Plane" object')
            return PlaneConverter.convert_to_ros_msg(obj)

        rospy.loginfo('[ae_convert_to_ros_msg] Unknown type; ignoring request')
        return None
