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

import abc

class ExecutionModelBase(object):
    def __init__(self, model_id=''):
        self.id = model_id

    @abc.abstractmethod
    def generate_data(self):
        pass

    @abc.abstractmethod
    def process_data(self, data):
        pass

    @abc.abstractmethod
    def input_to_dict(self):
        pass

    @abc.abstractmethod
    def result_to_dict(self, results):
        pass
