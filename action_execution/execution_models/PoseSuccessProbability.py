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

import numpy as np

from action_execution.execution_models.model import ExecutionModelBase

class PoseSuccessProbability(ExecutionModelBase):
    def __init__(self, candidates_and_probabilities,
                 action_id='', data_logger=None, **kwargs):
        super(PoseSuccessProbability, self).__init__(model_id='pose_success_probability')
        self.candidates_and_probabilities = candidates_and_probabilities

    def process_data(self):
        '''Generates a set of samples

        Keyword arguments:
        number_of_samples -- number of samples to generate

        Returns:
        candidate_poses -- a list of 'action_execution.geometry.pose.Pose3' objects

        '''

        pose_candidates = list()
        success_probabilities = list()

        for pose_dict in self.candidates_and_probabilities:
            pose_candidates.extend(pose_dict['candidate_poses'])
            success_probabilities.extend(pose_dict['success_probabilities'])
        success_probabilities = np.array(success_probabilities)

        sorting_indices = np.argsort(success_probabilities)[::-1]
        success_probabilities = success_probabilities[sorting_indices]
        success_probabilities /= np.sum(success_probabilities)
        pose_candidates = [pose_candidates[i] for i in sorting_indices]

        return {'candidate_poses': pose_candidates,
                'success_probabilities': success_probabilities}

    def input_to_dict(self):
        model_dict = {self.id: dict()}
        model_dict[self.id]['candidate_poses'] = list()
        model_dict[self.id]['success_probabilities'] = list()
        for pose_dict in self.candidates_and_probabilities:
            for pose in pose_dict['candidate_poses']:
                model_dict[self.id]['candidate_poses'].append(pose.to_dict())
            for prob in pose_dict['success_probabilities']:
                model_dict[self.id]['success_probabilities'].append(prob)
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
