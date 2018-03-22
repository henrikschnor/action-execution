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
from action_execution.extern.transformations import quaternion_from_euler

class ReachabilityCheckerModel(ExecutionModelBase):
    def __init__(self, pose_candidate_dict,
                 action_id='', data_logger=None, **kwargs):
        super(ReachabilityCheckerModel, self).__init__(model_id='reachability_checker')

        self.pose_candidates = pose_candidate_dict['candidate_poses']
        self.frame_id = ''
        self.arm_name = None
        self.arm = None

        if ExecutionConfigKeys.FRAME_ID in kwargs:
            self.frame_id = kwargs[ExecutionConfigKeys.FRAME_ID]
        if ExecutionConfigKeys.ARM in kwargs:
            self.arm_name = kwargs[ExecutionConfigKeys.ARM]
            self.arm = moveit_commander.MoveGroupCommander(self.arm_name)

    def process_data(self):
        pose_candidate = None
        reachable_pose_found = False
        pose_candidate_idx = 0
        while not reachable_pose_found and pose_counter <= len(self.pose_candidates):
            candidate = self.pose_candidates[pose_candidate_idx]

            pose = PoseStamped()
            pose.header.frame_id = candidate.frame_id
            pose.pose.position.x = candidate.position.x
            pose.pose.position.y = candidate.position.y
            pose.pose.position.y = candidate.position.z

            quaternion = quaternion_from_euler(candidate.orientation.x,
                                               candidate.orientation.y,
                                               candidate.orientation.z)

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
            if len(plan.joint_trajectory.points) > 0:
                reachable_pose_found = True
                pose_candidate = candidate
            else:
                pose_candidate_idx += 1

        return {'pose_candidate': pose_candidate, 'pose_candidate_idx': pose_candidate_idx}

    def input_to_dict(self):
        model_dict = {self.id: dict()}
        model_dict[self.id][ExecutionConfigKeys.FRAME_ID] = self.frame_id
        model_dict[self.id]['pose_candidates'] = list()
        for pose in self.pose_candidates:
            model_dict[self.id]['pose_candidates'].append(pose.to_dict())
        model_dict[self.id][ExecutionConfigKeys.ARM] = self.arm_name
        return model_dict

    def result_to_dict(self, results):
        result_dict = {self.id: dict()}
        result_dict[self.id]['pose_candidate'] = results['pose_candidate'].to_dict()
        result_dict[self.id]['pose_candidate_idx'] = results['pose_candidate_idx']
        return result_dict
