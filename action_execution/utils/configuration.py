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

from action_execution.geometry.pose import Pose3
from action_execution.geometry.object import Object3d

class Configuration(object):
    def __init__(self, frame_id='', static_objs=list(),
                 manipulated_obj=Object3d(), surface=Object3d()):
        self.frame_id = frame_id
        self.static_objs = static_objs
        self.manipulated_obj = manipulated_obj
        self.surface = surface
