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

class Plane(object):
    '''Defines a plane.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, obj_id='', pose=Pose3(), bbox=BBox3()):
        self.id = obj_id
        self.pose = deepcopy(pose)
        self.bbox = deepcopy(bbox)

    def __deepcopy__(self, memo):
        return Plane(self.id, deepcopy(self.pose), deepcopy(self.bbox))

    def to_dict(self):
        obj_dict = dict()

        obj_dict['header'] = dict()
        obj_dict['header']['type'] = 'Plane'

        obj_dict['id'] = self.id
        obj_dict['pose'] = self.pose.to_dict()
        obj_dict['bounding_box'] = self.bbox.to_dict()
        return obj_dict

    @staticmethod
    def from_dict(obj_dict):
        obj = Plane()
        obj.id = obj_dict['id']
        obj.pose = Pose3.from_dict(obj_dict['pose'])
        obj.bbox = BBox3.from_dict(obj_dict['bounding_box'])
        return obj
