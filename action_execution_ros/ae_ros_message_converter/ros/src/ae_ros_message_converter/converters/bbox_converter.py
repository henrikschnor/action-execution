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

        min_vec = Vector3(msg.vertices[0].x, msg.vertices[0].y, msg.vertices[0].z)
        max_vec = Vector3(msg.vertices[0].x, msg.vertices[0].y, msg.vertices[0].z)

        for vertex in msg.vertices:
            if vertex.x < min_vec.x:
                min_vec.x = vertex.x
            elif vertex.x > max_vec.x:
                max_vec.x = vertex.x

            if vertex.y < min_vec.y:
                min_vec.y = vertex.y
            elif vertex.y > max_vec.y:
                max_vec.y = vertex.y

            if vertex.z < min_vec.z:
                min_vec.z = vertex.z
            elif vertex.z > max_vec.z:
                max_vec.z = vertex.z

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

        point1 = Point(obj.min.x, obj.min.y, obj.min.z)
        point2 = Point(obj.min.x, obj.max.y, obj.min.z)
        point3 = Point(obj.max.x, obj.max.y, obj.min.z)
        point4 = Point(obj.max.x, obj.min.y, obj.min.z)
        point5 = Point(obj.min.x, obj.min.y, obj.max.z)
        point6 = Point(obj.min.x, obj.max.y, obj.max.z)
        point7 = Point(obj.max.x, obj.max.y, obj.max.z)
        point8 = Point(obj.max.x, obj.min.y, obj.max.z)

        bbox.vertices.append(point1)
        bbox.vertices.append(point2)
        bbox.vertices.append(point3)
        bbox.vertices.append(point4)
        bbox.vertices.append(point5)
        bbox.vertices.append(point6)
        bbox.vertices.append(point7)
        bbox.vertices.append(point8)

        return bbox
