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

from shapely.geometry import Polygon

from action_execution.geometry.vector import Vector3

class BBox3(object):
    '''Defines a 3D bounding box.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, min_values=Vector3(), max_values=Vector3()):
        self.min = deepcopy(min_values)
        self.max = deepcopy(max_values)

    def __deepcopy__(self, memo):
        return BBox3(deepcopy(self.min), deepcopy(self.max))

    def __str__(self):
        min_str = '  x: {0}\n  y: {1}\n  z: {2}'.format(self.min.x, self.min.y, self.min.z)
        max_str = '  x: {0}\n  y: {1}\n  z: {2}'.format(self.max.x, self.max.y, self.max.z)
        return 'min:\n{0}\nmax:\n{1}'.format(min_str, max_str)

    def rotate_around_z(self, rotation_centre, theta):
        '''Rotates the bounding around the z-axis by an angle 'theta'
        and 'rotation_centre' as the centre of rotation.

        Keyword arguments:
        rotation_centre -- centre of rotation
        theta -- a rotation angle in radians

        '''
        coordinates = self.get_coordinates()
        for i in xrange(len(coordinates)):
            coordinates[i] -= rotation_centre
            coordinates[i].rotate_around_z(theta)
            coordinates[i] += rotation_centre
        self.update_coordinates(coordinates)

    def get_z_projection(self):
        '''Returns a 'shapely.geometry.Polygon' object that represents
        the projection of the bounding box on the xy-plane.
        '''
        lower_left = (self.min.x, self.min.y)
        lower_right = (self.max.x, self.min.y)
        top_right = (self.max.x, self.max.y)
        top_left = (self.min.x, self.max.y)

        proj = Polygon((lower_left, lower_right, top_right, top_left))
        return proj

    def get_coordinates(self):
        '''Returns a list of 'action_execution.geometry.vector.Vector3' objects
        that represent the coordinates of the bounding box.
        '''
        point1 = Vector3(self.min.x, self.min.y, self.min.z)
        point2 = Vector3(self.min.x, self.min.y, self.max.z)
        point3 = Vector3(self.min.x, self.max.y, self.min.z)
        point4 = Vector3(self.min.x, self.max.y, self.max.z)
        point5 = Vector3(self.max.x, self.min.y, self.min.z)
        point6 = Vector3(self.max.x, self.min.y, self.max.z)
        point7 = Vector3(self.max.x, self.max.y, self.min.z)
        point8 = Vector3(self.max.x, self.max.y, self.max.z)
        points = [point1, point2, point3, point4, point5, point6, point7, point8]
        return points

    def update_coordinates(self, coordinates):
        '''Updates the bounding box based on the values in 'coordinates'.

        Keyword arguments:
        coordinates -- a list of 'action_execution.geometry.vector.Vector3' objects
                       that represent the updated coordinates of the bounding box

        '''
        min_x = coordinates[0].x
        max_x = coordinates[0].x
        min_y = coordinates[0].y
        max_y = coordinates[0].y
        min_z = coordinates[0].z
        max_z = coordinates[0].z

        for point in coordinates:
            if point.x < min_x:
                min_x = point.x
            elif point.x > max_x:
                max_x = point.x

            if point.y < min_y:
                min_y = point.y
            elif point.y > max_y:
                max_y = point.y

            if point.z < min_z:
                min_z = point.z
            elif point.z > max_z:
                max_z = point.z

        self.min.update_coordinates(min_x, min_y, min_z)
        self.max.update_coordinates(max_x, max_y, max_z)

    def planar_translate_to(self, old_position, new_position):
        '''Translates the bounding box to the specified position.

        Keyword arguments:
        old_position -- an 'action_execution.geometry.vector.Vector2' object
                        specifying the current planar position of the bounding box
        new_position -- an 'action_execution.geometry.vector.Vector2' object
                        specifying the new planar position of the bounding box

        '''
        max_pos_x_diff = self.max.x - old_position.x
        min_pos_x_diff = old_position.x - self.min.x
        self.max.x = new_position.x + max_pos_x_diff
        self.min.x = new_position.x - min_pos_x_diff

        max_pos_y_diff = self.max.y - old_position.y
        min_pos_y_diff = old_position.y - self.min.y
        self.max.y = new_position.y + max_pos_y_diff
        self.min.y = new_position.y - min_pos_y_diff

    def to_dict(self):
        obj_dict = dict()

        obj_dict['header'] = dict()
        obj_dict['header']['type'] = 'BBox3'

        obj_dict['min'] = self.min.to_dict()
        obj_dict['max'] = self.max.to_dict()
        return obj_dict

    @staticmethod
    def from_dict(obj_dict):
        obj = BBox3()
        obj.min = Vector3.from_dict(obj_dict['min'])
        obj.max = Vector3.from_dict(obj_dict['max'])
        return obj
