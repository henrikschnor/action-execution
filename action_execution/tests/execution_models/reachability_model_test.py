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

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from action_execution.geometry.vector import Vector3
from action_execution.geometry.bbox import BBox3
from action_execution.geometry.pose import Pose3
from action_execution.geometry.object import Object3d
from action_execution.execution_models.reachability import ReachabilityModel
from action_execution.logger.execution_data_logger import ExecutionDataLogger
from action_execution.config_keys import LoggerConfigKeys
from action_execution.test_utils.table_two_objects import *

def plot_points(surface, static_obj, poses):
    surface_corners = surface.get_z_projection().bounds
    static_obj_corners = list()
    for obj in static_obj:
        static_obj_corners.append(obj.get_z_projection().bounds)

    fig = plt.figure()
    axes = fig.add_subplot(111, aspect='equal')

    surface_coords = (surface_corners[0], surface_corners[1])
    surface_width = surface_corners[2] - surface_corners[0]
    surface_height = surface_corners[3] - surface_corners[1]
    axes.add_patch(patches.Rectangle(surface_coords, surface_width, surface_height))

    for i in xrange(len(static_obj_corners)):
        obj_coords = (static_obj_corners[i][0], static_obj_corners[i][1])
        obj_width = static_obj_corners[i][2] - static_obj_corners[i][0]
        obj_height = static_obj_corners[i][3] - static_obj_corners[i][1]
        axes.add_patch(patches.Rectangle(obj_coords, obj_width, obj_height, color='r'))

    for pose in poses:
        plt.plot(pose.position.x, pose.position.y, color='green', marker='o')

    plt.xlim([surface_coords[0] - 1., surface_coords[0] + surface_width + 1.])
    plt.ylim([surface_coords[1] - 1., surface_coords[1] + surface_height + 1.])
    plt.show()

if __name__ == '__main__':
    arm_name = 'arm'
    model = ReachabilityModel(frame_id=obj_config.frame_id,
                              manipulated_object=obj_config.manipulated_obj,
                              objects_on_surface=obj_config.static_objs,
                              surface=obj_config.surface,
                              arm=arm_name)

    number_of_samples = 10
    model_results = model.generate_data(number_of_samples)
    poses = model_results['candidate_poses']
    plot_points(obj_config.surface, obj_config.static_objs, poses)

    timestamp = int(round(time.time()) * 1000)
    data_logger = ExecutionDataLogger(LoggerConfigKeys.DB_NAME_DEBUG)

    input_log_dict = {'action_name': '', 'timestamp': timestamp}
    result_log_dict = {'action_name': '', 'timestamp': timestamp}
    input_log_id = data_logger.log_metadata(LoggerConfigKeys.MODEL_INPUT_COLLECTION_DEBUG,
                                            input_log_dict)
    result_log_id = data_logger.log_metadata(LoggerConfigKeys.MODEL_RESULT_COLLECTION_DEBUG,
                                             result_log_dict)

    data_logger.log_model_data(LoggerConfigKeys.MODEL_INPUT_COLLECTION_DEBUG,
                               model.input_to_dict(), input_log_id)
    data_logger.log_model_data(LoggerConfigKeys.MODEL_RESULT_COLLECTION_DEBUG,
                               model.result_to_dict(model_results), result_log_id)
