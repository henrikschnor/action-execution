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

from action_execution.geometry.pose import Pose3
from action_execution.geometry.bbox import BBox3

class Object3d(object):
    '''Defines an object in space.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, obj_id='', obj_type='',
                 pose=Pose3(), bbox=BBox3()):
        self.id = obj_id
        self.type = obj_type
        self.pose = pose
        self.bbox = bbox

    def __deepcopy__(self, memo):
        return Object3d(self.id, self.type, deepcopy(self.pose), deepcopy(self.bbox))

    def rotate_around_z(self, theta):
        '''Rotates the object around the z-axis by an angle 'theta'.

        Keyword arguments:
        theta -- a rotation angle in radians

        '''
        self.pose.orientation.z = theta
        self.bbox.rotate_around_z(self.pose.position, theta)

    def get_z_projection(self):
        '''Returns a 'shapely.geometry.Polygon' object that represents
        the projection of the bounding box on the xy-plane.
        '''
        return self.bbox.get_z_projection()

    def planar_translate_to(self, new_position):
        '''Translates the object to the specified position.

        Keyword arguments:
        new_position -- an 'action_execution.geometry.vector.Vector2' object

        '''
        old_position = deepcopy(self.pose.position)
        self.pose.position.x = new_position.x
        self.pose.position.y = new_position.y
        self.bbox.planar_translate_to(old_position, new_position)

    def to_dict(self):
        obj_dict = dict()

        obj_dict['header'] = dict()
        obj_dict['header']['type'] = 'Object3d'

        obj_dict['id'] = self.id
        obj_dict['type'] = self.type
        obj_dict['pose'] = self.pose.to_dict()
        obj_dict['bounding_box'] = self.bbox.to_dict()
        return obj_dict

    @staticmethod
    def from_dict(obj_dict):
        obj = Object3d()
        obj.id = obj_dict['id']
        obj.type = obj_dict['type']
        obj.pose = Pose3.from_dict(obj_dict['pose'])
        obj.bbox = BBox3.from_dict(obj_dict['bounding_box'])
        return obj
