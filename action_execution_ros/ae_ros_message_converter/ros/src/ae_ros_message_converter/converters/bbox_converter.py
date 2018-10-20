#!/usr/bin/env python

from geometry_msgs.msg import Point
from mcr_perception_msgs.msg import BoundingBox

from ae_ros_message_converter.converters.converter_base import ConverterBase

from action_execution.geometry.bbox import BBox3, Vector3

class BoundingBoxConverter(ConverterBase):
    def __init__(self):
        super(BoundingBoxConverter, self).__init__()

    @staticmethod
    def convert_ros_msg(msg):
        '''Converts an 'mcr_perception_msgs.msg.BoundingBox' message to an
        'action_execution.geometry.bbox.BBox3' object.

        Keyword argumens:
        msg -- an 'mcr_perception_msgs.msg.BoundingBox' message

        '''
        bbox = BBox3()

        min_vec = Vector3(msg.center.x - (msg.dimensions.x / 2.),
                          msg.center.y - (msg.dimensions.y / 2.),
                          msg.center.z - (msg.dimensions.z / 2.))
        max_vec = Vector3(msg.center.x + (msg.dimensions.x / 2.),
                          msg.center.y + (msg.dimensions.y / 2.),
                          msg.center.z + (msg.dimensions.z / 2.))

        bbox.min = min_vec
        bbox.max = max_vec

        return bbox

    @staticmethod
    def convert_to_ros_msg(obj):
        '''Converts an 'action_execution.geometry.bbox.BBox3' object to an
        'mcr_perception_msgs.msg.BoundingBox' message.

        Keyword argumens:
        obj -- an 'action_execution.geometry.bbox.BBox3' object

        '''
        bbox = BoundingBox()

        bbox.center.x = (obj.max.x + obj.min.x) / 2.
        bbox.center.y = (obj.max.y + obj.min.y) / 2.
        bbox.center.z = (obj.max.z + obj.min.z) / 2.

        bbox.dimensions.x = obj.max.x - obj.min.x
        bbox.dimensions.y = obj.max.y - obj.min.y
        bbox.dimensions.z = obj.max.z - obj.min.z

        return bbox
