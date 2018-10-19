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

from action_execution.geometry.vector import Vector3
from action_execution.geometry.bbox import BBox3
from action_execution.geometry.pose import Pose3
from action_execution.geometry.object import Object3d
from action_execution.geometry.plane import Plane
from action_execution.utils.configuration import Configuration

obj_config = Configuration()
obj_config.frame_id = 'odom'

static_obj1 = Object3d(obj_type='ketchup_bottle',
                       pose=Pose3(frame_id='odom',
                                  position=Vector3(-0.75, -0.25, 0.75),
                                  orientation=Vector3(0., 0., 0.)),
                       bbox=BBox3(Vector3(-0.789, -0.289, 0.75),
                                  Vector3(-0.711, -0.211, 0.958)))
static_obj2 = Object3d(obj_type='ketchup_bottle',
                       pose=Pose3(frame_id='odom',
                                  position=Vector3(-1., -0.1, 0.75),
                                  orientation=Vector3(0., 0., 0.)),
                       bbox=BBox3(Vector3(-1.039, -0.139, 0.75),
                                  Vector3(-0.961, -0.061, 0.958)))
static_obj3 = Object3d(obj_type='book',
                       pose=Pose3(frame_id='odom',
                                  position=Vector3(-1., 0.5, 0.75),
                                  orientation=Vector3(0., 0., 1.57)),
                       bbox=BBox3(Vector3(-1.225, 0.635, 0.75),
                                  Vector3(-0.775, 0.365, 0.798)))

obj_config.static_objs = [static_obj1, static_obj2, static_obj3]

obj_config.manipulated_obj = Object3d(pose=Pose3(frame_id='odom',
                                                 position=Vector3(-0.126, 0.190, 0.851),
                                                 orientation=Vector3(0., 0., 0.)),
                                      bbox=BBox3(Vector3(-0.164, 0.151, 0.849),
                                                 Vector3(-0.087, 0.229, 1.057)))

obj_config.surface = Plane(pose=Pose3(frame_id='odom',
                                      position=Vector3(-1., 0., 0.75)),
                           bbox=BBox3(Vector3(-1.3, -0.7, 0.),
                                      Vector3(-0.7, 0.7, 0.75)))
