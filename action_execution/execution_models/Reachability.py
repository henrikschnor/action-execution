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
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped, Quaternion

from action_execution.config_keys import ExecutionConfigKeys
from action_execution.execution_models.model import ExecutionModelBase
from action_execution.geometry.vector import Vector2, Vector3
from action_execution.geometry.pose import Pose3
from action_execution.extern.transformations import quaternion_from_euler

class Reachability(ExecutionModelBase):
    def __init__(self, action_id='', data_logger=None, **kwargs):
        super(Reachability, self).__init__(model_id='reachability')

        self.frame_id = ''
        self.manipulated_object = None
        self.objects = list()
        self.surface = None
        self.arm_name = None
        self.arm = None

        if ExecutionConfigKeys.FRAME_ID in kwargs:
            self.frame_id = kwargs[ExecutionConfigKeys.FRAME_ID]
        if ExecutionConfigKeys.MANIPULATED_OBJECT in kwargs:
            self.manipulated_object = kwargs[ExecutionConfigKeys.MANIPULATED_OBJECT]
        if ExecutionConfigKeys.OBJECTS_ON_SURFACE in kwargs:
            self.objects = kwargs[ExecutionConfigKeys.OBJECTS_ON_SURFACE]
        if ExecutionConfigKeys.SURFACE in kwargs:
            self.surface = kwargs[ExecutionConfigKeys.SURFACE]
        if ExecutionConfigKeys.ARM in kwargs:
            self.arm_name = kwargs[ExecutionConfigKeys.ARM]
            self.arm = moveit_commander.MoveGroupCommander(self.arm_name)

    def generate_data(self, number_of_samples):
        candidate_poses = list()

        # we get the z-projections of the objects that are already on the surface
        object_polygons = list()
        for obj in self.objects:
            polygon = obj.get_z_projection()
            object_polygons.append(polygon)

        # we generate samples in the free space on the surface, i.e.
        # we ignore samples that cause collisions with the other objects
        collected_samples = 0
        while collected_samples < number_of_samples:
            obj_copy = deepcopy(self.manipulated_object)

            # we generate a random position on the surface
            position = Vector2.random_vector(self.surface.bbox.min, self.surface.bbox.max)
            obj_copy.planar_translate_to(position)

            # we generate a random orientation around z;
            # we don't rotate the object around the other axes
            z_orientation = np.random.uniform(0., 2.*np.pi)
            obj_copy.rotate_around_z(z_orientation)

            # we check if the object would collide with any of the other
            # objects in the newly generated pose
            manipulated_object_polygon = obj_copy.get_z_projection()
            reachable = True

            pose = PoseStamped()
            pose.header.frame_id = self.manipulated_object.pose.frame_id
            pose.pose.position.x = position.x
            pose.pose.position.y = position.y

            # TODO: change this to something that makes more sense
            pose.pose.position.z = self.surface.pose.position.z + 0.1

            quaternion = quaternion_from_euler(obj_copy.pose.orientation.x,
                                               obj_copy.pose.orientation.y,
                                               z_orientation)

            quaternion_msg = Quaternion()
            quaternion_msg.x = quaternion[0]
            quaternion_msg.y = quaternion[1]
            quaternion_msg.z = quaternion[2]
            quaternion_msg.w = quaternion[3]

            pose.pose.orientation = quaternion_msg

            self.arm.set_pose_reference_frame(self.manipulated_object.pose.frame_id)
            self.arm.clear_pose_targets()
            self.arm.set_pose_target(pose.pose)
            plan = self.arm.plan()
            reachable = False
            if len(plan.joint_trajectory.points) > 0:
                reachable = True

            # we take the generated pose as a valid candidate if the
            # pose is reachable by the manipulator
            if reachable:
                position = Vector3(position.x,
                                   position.y,
                                   self.surface.pose.position.z)
                orientation = Vector3(obj_copy.pose.orientation.x,
                                      obj_copy.pose.orientation.y,
                                      z_orientation)
                pose = Pose3(self.manipulated_object.pose.frame_id, position, orientation)
                candidate_poses.append(pose)
                collected_samples += 1

        return {'candidate_poses': candidate_poses}

    def input_to_dict(self):
        model_dict = {self.id: dict()}
        model_dict[self.id][ExecutionConfigKeys.FRAME_ID] = self.frame_id
        model_dict[self.id][ExecutionConfigKeys.MANIPULATED_OBJECT] = self.manipulated_object.to_dict()
        model_dict[self.id][ExecutionConfigKeys.OBJECTS_ON_SURFACE] = list()
        for obj in self.objects:
            model_dict[self.id][ExecutionConfigKeys.OBJECTS_ON_SURFACE].append(obj.to_dict())
        model_dict[self.id][ExecutionConfigKeys.SURFACE] = self.surface.to_dict()
        model_dict[self.id][ExecutionConfigKeys.ARM] = self.arm_name
        return model_dict

    def result_to_dict(self):
        result_dict = {self.id: dict()}
        result_dict[self.id]['candidate_poses'] = list()
        for pose in results['candidate_poses']:
            result_dict[self.id]['candidate_poses'].append(pose.to_dict())
        return result_dict
