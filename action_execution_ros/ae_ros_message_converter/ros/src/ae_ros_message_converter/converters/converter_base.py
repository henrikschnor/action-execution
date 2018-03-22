#!/usr/bin/env python

import abc

class ConverterBase(object):
    def __init__(self):
        pass

    @abc.abstractmethod
    @staticmethod
    def convert_ros_msg(msg):
        pass

    @abc.abstractmethod
    @staticmethod
    def convert_to_ros_msg(obj):
        pass
