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

from action_execution.config_keys import ExecutionConfigKeys
from action_execution.execution_models.model import ExecutionModelBase
from action_execution.geometry.vector import Vector2, Vector3
from action_execution.geometry.pose import Pose3

class ProximityModel(ExecutionModelBase):
    def __init__(self, action_id='', data_logger=None, **kwargs):
        super(ProximityModel, self).__init__(model_id='proximity')

        self.frame_id = ''
        self.manipulated_object = None
        self.objects = list()
        self.surface = None
        self.robot_pose = None

        if ExecutionConfigKeys.FRAME_ID in kwargs:
            self.frame_id = kwargs[ExecutionConfigKeys.FRAME_ID]
        if ExecutionConfigKeys.MANIPULATED_OBJECT in kwargs:
            self.manipulated_object = kwargs[ExecutionConfigKeys.MANIPULATED_OBJECT]
        if ExecutionConfigKeys.OBJECTS_ON_SURFACE in kwargs:
            self.objects = kwargs[ExecutionConfigKeys.OBJECTS_ON_SURFACE]
        if ExecutionConfigKeys.SURFACE in kwargs:
            self.surface = kwargs[ExecutionConfigKeys.SURFACE]
        if ExecutionConfigKeys.ROBOT_POSE in kwargs:
            self.robot_pose = kwargs[ExecutionConfigKeys.ROBOT_POSE]

    def generate_data(self, number_of_samples):
        '''Generates a set of samples

        Keyword arguments:
        number_of_samples -- number of samples to generate

        Returns:
        candidate_poses -- a list of 'action_execution.geometry.pose.Pose3' objects

        '''
        candidate_poses = list()

        # we get the z-projections of the objects that are already on the surface
        object_polygons = list()
        for obj in self.objects:
            polygon = obj.get_z_projection()
            object_polygons.append(polygon)

        # we generate samples in the free space on the surface, i.e.
        # we ignore samples that cause collisions with the other objects
        collected_samples = 0
        distances = np.zeros(number_of_samples)
        while collected_samples < number_of_samples:
            obj_copy = deepcopy(self.manipulated_object)

            # we generate a random position on the surface
            position = Vector2.random_vector(self.surface.bbox.min, self.surface.bbox.max)
            obj_copy.planar_translate_to(position)

            # we generate a random orientation around z;
            # we don't rotate the object around the other axes
            z_orientation = np.random.uniform(0., 2.*np.pi)
            obj_copy.rotate_around_z(z_orientation)

            # we take the generated pose as a valid candidate if the
            # object doesn't collide with any of the objects on the surface
            position = Vector3(position.x,
                               position.y,
                               obj_copy.bbox.min.z)
            orientation = Vector3(obj_copy.pose.orientation.x,
                                  obj_copy.pose.orientation.y,
                                  z_orientation)
            pose = Pose3(self.manipulated_object.pose.frame_id, position, orientation)
            candidate_poses.append(pose)

            distances[collected_samples] = position.distance(self.robot_pose.position)
            collected_samples += 1

        # we value closer poses higher than farther ones;
        # the results are treated as success probabilities,
        # though the values actually encode preference
        success_probabilities = np.max(distances) / distances
        success_probabilities /= np.sum(success_probabilities)

        return {'candidate_poses': candidate_poses,
                'success_probabilities': success_probabilities}

    def process_data(self, data):
        '''Generates a set of samples

        Keyword arguments:
        data -- number of samples to generate

        Returns:
        candidate_poses -- a list of 'action_execution.geometry.pose.Pose3' objects

        '''
        candidate_poses = data['candidate_poses']
        success_probabilities = np.array(data['success_probabilities'])

        distances = np.zeros(len(candidate_poses))
        for i, pose in enumerate(candidate_poses):
            distances[i] = pose.position.distance(self.robot_pose.position)

        # we value closer poses higher than farther ones;
        # the results are treated as success probabilities,
        # though the values actually encode preference
        distance_success_probabilities = np.max(distances) / distances

        success_probabilities *= distance_success_probabilities
        success_probabilities /= np.sum(success_probabilities)

        return {'candidate_poses': candidate_poses,
                'success_probabilities': success_probabilities}

    def input_to_dict(self):
        model_dict = {self.id: dict()}
        model_dict[self.id]['frame_id'] = self.frame_id
        model_dict[self.id]['manipulated_object'] = self.manipulated_object.to_dict()
        model_dict[self.id]['static_objects'] = list()
        for obj in self.objects:
            model_dict[self.id]['static_objects'].append(obj.to_dict())
        model_dict[self.id]['surface'] = self.surface.to_dict()
        model_dict[self.id]['robot_pose'] = self.robot_pose.to_dict()
        return model_dict

    def result_to_dict(self, results):
        result_dict = {self.id: dict()}
        result_dict[self.id]['candidate_poses'] = list()
        result_dict[self.id]['success_probabilities'] = list()
        for pose in results['candidate_poses']:
            result_dict[self.id]['candidate_poses'].append(pose.to_dict())
        for prob in results['success_probabilities']:
            result_dict[self.id]['success_probabilities'].append(prob)
        return result_dict
