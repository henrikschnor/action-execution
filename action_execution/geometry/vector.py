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

from __future__ import print_function
import numpy as np
from copy import deepcopy

class Vector2(object):
    '''Defines a 2D vector.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, x=0., y=0.):
        self.vec = np.array([x, y])[np.newaxis].T

    def __get_x(self):
        '''Returns the vector's first coordinate.
        '''
        return self.vec[0][0]

    def __set_x(self, val):
        '''Sets the vector's first coordinate.
        '''
        self.vec[0][0] = val

    def __get_y(self):
        '''Returns the vector's second coordinate.
        '''
        return self.vec[1][0]

    def __set_y(self, val):
        '''Sets the vector's second coordinate.
        '''
        self.vec[1][0] = val

    def __deepcopy__(self, memo):
        return Vector2(self.x, self.y)

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    x = property(__get_x, __set_x)
    y = property(__get_y, __set_y)

    def update_coordinates(self, x_val, y_val):
        '''Updates all vector coordinates.

        Keyword arguments:
        x_val -- the new value of the first argument
        y_val -- the new value of the second argument

        '''
        self.x = x_val
        self.y = y_val

    @staticmethod
    def random_vector(lower_limits, upper_limits):
        '''Returns a vector whose coordinates are between
        the values specified by 'lower_limits' and 'upper_limits'.

        Keyword arguments:
        lower_limits -- a Vector2 object specifying the lower coordinate limits of the new vector
        upper_limits -- a Vector2 object specifying the upper coordinate limits of the new vector

        '''
        x = np.random.uniform(lower_limits.x, upper_limits.x)
        y = np.random.uniform(lower_limits.y, upper_limits.y)
        return Vector2(x, y)

    def distance(self, other):
        self_array = np.array([self.x, self.y])
        other_array = np.array([other.x, other.y])
        return np.linalg.norm(self_array - other_array)

    def to_dict(self):
        obj_dict = dict()

        obj_dict['header'] = dict()
        obj_dict['header']['type'] = 'Vector2'

        obj_dict['x'] = self.x
        obj_dict['y'] = self.y

        return obj_dict

    @staticmethod
    def from_dict(obj_dict):
        obj = Vector2()
        obj.x = obj_dict['x']
        obj.y = obj_dict['y']
        return obj

    def print_vec(self):
        print('x = {0}, y = {1}'.format(self.x, self.y))

class Vector3(object):
    '''Defines a 3D vector.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, x=0., y=0., z=0.):
        self.vec = np.array([x, y, z])[np.newaxis].T

    def __get_x(self):
        '''Returns the vector's first coordinate.
        '''
        return self.vec[0][0]

    def __set_x(self, val):
        '''Sets the vector's first coordinate.
        '''
        self.vec[0][0] = val

    def __get_y(self):
        '''Returns the vector's second coordinate.
        '''
        return self.vec[1][0]

    def __set_y(self, val):
        '''Sets the vector's first coordinate.
        '''
        self.vec[1][0] = val

    def __get_z(self):
        '''Returns the vector's third coordinate.
        '''
        return self.vec[2][0]

    def __set_z(self, val):
        '''Sets the vector's first coordinate.
        '''
        self.vec[2][0] = val

    def __deepcopy__(self, memo):
        return Vector3(self.x, self.y, self.z)

    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    x = property(__get_x, __set_x)
    y = property(__get_y, __set_y)
    z = property(__get_z, __set_z)

    def update_coordinates(self, x_val, y_val, z_val):
        '''Updates all vector coordinates.

        Keyword arguments:
        x_val -- the new value of the first argument
        y_val -- the new value of the second argument
        z_val -- the new value of the third argument

        '''
        self.x = x_val
        self.y = y_val
        self.z = z_val

    def rotate_around_z(self, theta):
        '''Rotates the vector around the z-axis by an angle 'theta'.

        Keyword arguments:
        theta -- a rotation angle in radians

        '''
        rot_z = Vector3.__get_z_rotation(theta)
        self.vec = rot_z.dot(self.vec)

    @staticmethod
    def __get_z_rotation(theta):
        '''Returns a rotation matrix around the z-axis.

        Keyword arguments:
        theta -- a rotation angle in radians

        '''
        rot = np.array([[np.cos(theta), -np.sin(theta), 0.],
                        [np.sin(theta), np.cos(theta), 0.],
                        [0., 0., 1.]])
        return rot

    @staticmethod
    def random_vector(lower_limits, upper_limits):
        '''Returns a vector whose coordinates are between
        the values specified by 'lower_limits' and 'upper_limits'.

        Keyword arguments:
        lower_limits -- a Vector2 object specifying the lower coordinate limits of the new vector
        upper_limits -- a Vector2 object specifying the upper coordinate limits of the new vector

        '''
        x = np.random.uniform(lower_limits.x, upper_limits.x)
        y = np.random.uniform(lower_limits.y, upper_limits.y)
        z = np.random.uniform(lower_limits.z, upper_limits.z)
        return Vector3(x, y, z)

    def distance(self, other):
        self_array = np.array([self.x, self.y, self.z])
        other_array = np.array([other.x, other.y, self.z])
        return np.linalg.norm(self_array - other_array)

    def to_dict(self):
        obj_dict = dict()

        obj_dict['header'] = dict()
        obj_dict['header']['type'] = 'Vector3'

        obj_dict['x'] = self.x
        obj_dict['y'] = self.y
        obj_dict['z'] = self.z
        return obj_dict

    @staticmethod
    def from_dict(obj_dict):
        obj = Vector3()
        obj.x = obj_dict['x']
        obj.y = obj_dict['y']
        obj.z = obj_dict['z']
        return obj

    def print_vec(self):
        print('x = {0}, y = {1}, z = {2}'.format(self.x, self.y, self.z))
