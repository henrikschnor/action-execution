'''
    Copyright 2018 by Alex Mitrevski <aleksandar.mitrevski@h-brs.de>

    This file is part of action-execution.

    action-execution is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    action-execution is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with action-execution. If not, see <http://www.gnu.org/licenses/>.
'''

from copy import deepcopy

from action_execution.geometry.vector import Vector2, Vector3

class Pose2(object):
    '''Defines a 2D pose.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, frame_id='', position=Vector2(), orientation=0.):
        '''Keyword arguments:

        position -- an 'action_execution.geometry.vector.Vector2' object
        orientation -- the orientation in radians

        '''
        self.frame_id = frame_id
        self.position = deepcopy(position)
        self.orientation = orientation

    def __deepcopy__(self, memo):
        return Pose2(self.frame_id, deepcopy(self.position), self.orientation)

    def to_dict(self):
        obj_dict = dict()

        obj_dict['header'] = dict()
        obj_dict['header']['type'] = 'Pose2'

        obj_dict['position'] = self.position.to_dict()
        obj_dict['orientation'] = self.orientation
        return obj_dict

    @staticmethod
    def from_dict(obj_dict):
        obj = Pose2()
        obj.frame_id = obj_dict['frame_id']
        obj.position = Vector2.from_dict(obj_dict['position'])
        obj.orientation = obj_dict['orientation']
        return obj

class Pose3(object):
    '''Defines a 3D pose.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, frame_id='', position=Vector3(), orientation=Vector3()):
        '''Keyword arguments:

        position -- an 'action_execution.geometry.vector.Vector3' object
        orientation -- an 'action_execution.geometry.vector.Vector3' object

        '''
        self.frame_id = frame_id
        self.position = deepcopy(position)
        self.orientation = deepcopy(orientation)

    def __deepcopy__(self, memo):
        return Pose3(self.frame_id, deepcopy(self.position), deepcopy(self.orientation))

    def to_dict(self):
        obj_dict = dict()

        obj_dict['header'] = dict()
        obj_dict['header']['type'] = 'Pose3'

        obj_dict['frame_id'] = self.frame_id
        obj_dict['position'] = self.position.to_dict()
        obj_dict['orientation'] = self.orientation.to_dict()
        return obj_dict

    @staticmethod
    def from_dict(obj_dict):
        obj = Pose3()
        obj.frame_id = obj_dict['frame_id']
        obj.position = Vector3.from_dict(obj_dict['position'])
        obj.orientation = Vector3.from_dict(obj_dict['orientation'])
        return obj
