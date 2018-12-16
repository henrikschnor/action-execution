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
import time
from os import listdir
from os.path import join, isfile
from importlib import import_module
import yaml

from action_execution.config_keys import PathConfig, ActionConfigKeys, \
                                         LoggerConfigKeys, MODEL_CLASS_MAPPING, \
                                         OBJ_MODULE_MAPPING
from action_execution.execution_model import ExecutionModel
from action_execution.failure_case import FailureCase
from action_execution.logger.execution_data_logger import ExecutionDataLogger

class ActionData(object):
    def __init__(self, action_name=None):
        self.id = action_name
        self.instance_of = list()
        self.capabilities = list()

class FailureCaseData(object):
    def __init__(self):
        self.keys = list()
        self.config = list()
        self.failure_cases = list()

class ModelData(object):
    def __init__(self):
        self.keys = list()
        self.config = list()
        self.models = list()
        self.sequence_models = False
        self.combination_model_key = None
        self.combination_model = None
        self.output_model_key = None
        self.output_model = None

class Action(object):
    '''A description of an action that can be used for making
    informed execution-related decisions; this should increase
    the likelihood that the action will be executed successfully.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, action_name=None):
        self.action_config_path = join(PathConfig.DIR_ABS_PATH,
                                       PathConfig.ACTION_CONFIG_PATH)
        self.failure_case_config_path = join(PathConfig.DIR_ABS_PATH,
                                             PathConfig.FAILURE_CASE_CONFIG_PATH)

        self.action_data = ActionData(action_name)
        self.failure_case_data = FailureCaseData()
        self.model_data = ModelData()
        self.data_logger = ExecutionDataLogger()

        if self.action_data.id is not None:
            self.load_config(self.action_data.id)

    def get_execution_guidelines(self, data_count=1, **kwargs):
        '''Gets the results from all action execution models.
        '''
        timestamp = int(round(time.time()) * 1000)
        log_dict = {'action_name': self.action_data.id,
                    'timestamp': timestamp,
                    'inputs': {'model_data': {}},
                    'outputs': {'model_data': {}}
                   }

        model_results = list()
        for i in range(len(self.model_data.models)):
            # we initialise the execution model
            model = self.model_data.models[i](**kwargs)
            log_dict['inputs']['model_data'].update(model.input_to_dict())
            valid_input, message = self.model_data.config[i].verify_input(kwargs)
            if valid_input:
                results = None
                if self.model_data.sequence_models and model_results:
                    results = model.process_data(model_results[0])
                    model_results[0] = results
                else:
                    results = model.generate_data(data_count)
                    model_results.append(results)
                log_dict['outputs']['model_data'].update(model.result_to_dict(results))
            else:
                print(self.model_data.keys[i] + ': ' + message)

        if self.model_data.combination_model is not None:
            # we initialise the combination model
            model = self.model_data.combination_model(model_results)
            model_results = model.process_data()
            log_dict['inputs']['model_data'].update(model.input_to_dict())
            log_dict['outputs']['model_data'].update(model.result_to_dict(results))
        else:
            model_results = model_results[0]

        if self.model_data.output_model is not None:
            # we initialise the output model
            model = self.model_data.output_model(model_results)
            model_results = model.process_data()
            log_dict['inputs']['model_data'].update(model.input_to_dict())
            log_dict['outputs']['model_data'].update(model.result_to_dict(results))

        self.data_logger.log_model_data(self.action_data.id, log_dict)
        return model_results

    def check_failures(self, **kwargs):
        '''Checks whether any of the known action failures have occurred.
        '''
        pass

    def get_execution_data(self, start_timestamp=0., end_timestamp=0.):
        # TODO: preprocess the input data before returning it
        input_data = self.data_logger.get_action_data(LoggerConfigKeys.MODEL_INPUT_COLLECTION,
                                                      self.action_data.id,
                                                      start_timestamp,
                                                      end_timestamp)

        result_data = self.data_logger.get_action_data(LoggerConfigKeys.MODEL_RESULT_COLLECTION,
                                                       self.action_data.id,
                                                       start_timestamp,
                                                       end_timestamp)

        result_data_dict = dict()
        for model, data in result_data.items():
            result_data_dict[model] = dict()
            for key, value in data.items():
                if isinstance(value, list):
                    result_data_dict[model][key] = list()
                    for v in value:
                        if isinstance(v, dict):
                            obj_type = v['header']['type']
                            obj_module = 'action_execution.' + OBJ_MODULE_MAPPING[obj_type]
                            obj_class = getattr(import_module(obj_module),
                                                obj_type)
                            result_data_dict[model][key].append(obj_class.from_dict(v))
                        else:
                            result_data_dict[model][key].append(v)

        return input_data, result_data_dict

    def load_config(self, action_name):
        '''Loads the config file of the action specified by 'action_name'

        Keyword arguments:
        action_name -- id of an action

        '''
        config_path = join(self.action_config_path, action_name + '.yaml')

        try:
            file_handle = open(config_path, 'r')
            config = yaml.load(file_handle)
            file_handle.close()

            self.action_data.id = action_name
            if ActionConfigKeys.INSTANCE_OF in config:
                self.action_data.instance_of = config[ActionConfigKeys.INSTANCE_OF]

            for action in self.action_data.instance_of:
                self.__load_parent_action_config(action)

            if ActionConfigKeys.CAPABILITIES in config:
                capabilities = config[ActionConfigKeys.CAPABILITIES]
                for capability in capabilities:
                    if capability not in self.action_data.capabilities:
                        self.action_data.capabilities.append(capability)

            if ActionConfigKeys.FAILURE_CASES in config:
                failure_cases = config[ActionConfigKeys.FAILURE_CASES]
                for failure in failure_cases:
                    if failure not in self.failure_case_data.keys:
                        self.failure_case_data.keys.append(failure)

            if ActionConfigKeys.MODELS in config:
                models = config[ActionConfigKeys.MODELS]
                for model in models:
                    if model not in self.model_data.keys:
                        self.model_data.keys.append(model)

            if ActionConfigKeys.SEQUENCE_MODELS in config:
                self.model_data.sequence_models = config[ActionConfigKeys.SEQUENCE_MODELS]

            if ActionConfigKeys.COMBINATION_MODEL in config:
                self.model_data.combination_model_key = config[ActionConfigKeys.COMBINATION_MODEL]

            if ActionConfigKeys.OUTPUT_MODEL in config:
                self.model_data.output_model_key = config[ActionConfigKeys.OUTPUT_MODEL]

            # for each model key, we create an instance of the
            # model config class and the model
            for model_key in self.model_data.keys:
                self.model_data.config.append(ExecutionModel(model_key))

                model_module = 'action_execution.execution_models.' + model_key
                model_class = getattr(import_module(model_module),
                                      MODEL_CLASS_MAPPING[model_key])
                self.model_data.models.append(model_class)

            if self.model_data.combination_model_key is not None:
                model_key = self.model_data.combination_model_key
                model_module = 'action_execution.execution_models.' + model_key
                model_class = getattr(import_module(model_module),
                                      MODEL_CLASS_MAPPING[model_key])
                self.model_data.combination_model = model_class

            if self.model_data.output_model_key is not None:
                model_key = self.model_data.output_model_key
                model_module = 'action_execution.execution_models.' + model_key
                model_class = getattr(import_module(model_module),
                                      MODEL_CLASS_MAPPING[model_key])
                self.model_data.output_model = model_class

            for failure_case in self.failure_case_data.keys:
                self.failure_case_data.config.append(FailureCase(failure_case))

            print('Description of "{0}" loaded successfully'.format(self.action_data.id))
        except IOError:
            print('Cannot load config of unknown action "{0}"'.format(self.action_data.id))

    def __load_parent_action_config(self, action_id):
        '''Loads the config files of the parents of the current action
        and adds their capabilities, known failure cases, and execution models
        to the current action's respective lists

        Keyword arguments:
        action_id

        '''
        config_path = join(self.action_config_path, action_id + '.yaml')
        file_handle = open(config_path, 'r')
        config = yaml.load(file_handle)
        file_handle.close()

        instance_of = list()
        if ActionConfigKeys.INSTANCE_OF in config:
            instance_of = config[ActionConfigKeys.INSTANCE_OF]

        if ActionConfigKeys.CAPABILITIES in config:
            capabilities = config[ActionConfigKeys.CAPABILITIES]
            for capability in capabilities:
                if capability not in self.action_data.capabilities:
                    self.action_data.capabilities.insert(0, capability)

        if ActionConfigKeys.FAILURE_CASES in config:
            failure_cases = config[ActionConfigKeys.FAILURE_CASES]
            for failure in failure_cases:
                if failure not in self.failure_case_data.keys:
                    self.failure_case_data.keys.insert(0, failure)

        if ActionConfigKeys.MODELS in config:
            models = config[ActionConfigKeys.MODELS]
            for model in models:
                if model not in self.model_data.keys:
                    self.model_data.keys.insert(0, model)

        if ActionConfigKeys.SEQUENCE_MODELS in config:
            self.model_data.sequence_models = config[ActionConfigKeys.SEQUENCE_MODELS]

        if ActionConfigKeys.COMBINATION_MODEL in config:
            self.model_data.combination_model_key = config[ActionConfigKeys.COMBINATION_MODEL]

        if ActionConfigKeys.OUTPUT_MODEL in config:
            self.model_data.output_model_key = config[ActionConfigKeys.OUTPUT_MODEL]

        for action in instance_of:
            self.__load_parent_action_config(action)

    def parent_actions(self):
        '''Returns a list of keys of the actions that the current action is an instance of
        '''
        return list(self.action_data.instance_of)

    def children_actions(self):
        '''Returns a list of keys of the actions that are direct instances of the current action
        '''
        action_list = list()
        for f_name in listdir(self.action_config_path):
            action_id = f_name.split('.')[0]
            f_path = join(self.action_config_path, f_name)
            if not isfile(f_path):
                continue
            file_handle = open(f_path, 'r')
            action_config = yaml.load(file_handle)
            file_handle.close()

            instance_of_keys = list()
            if ActionConfigKeys.INSTANCE_OF in action_config:
                instance_of_keys = action_config[ActionConfigKeys.INSTANCE_OF]

            if self.action_data.id in instance_of_keys:
                action_list.append(action_id)

        return action_list

    def print_config(self):
        '''Prints the values of the action's config fields
        '''
        print('action_id = {0}'.format(self.action_data.id))
        print('instance_of = {0}'.format(self.action_data.instance_of))
        print('capabilities = {0}'.format(self.action_data.capabilities))
        print('failure_case_keys = {0}'.format(self.failure_case_data.keys))
        print('model_keys = {0}'.format(self.model_data.keys))
        print('sequence_models = {0}'.format(self.model_data.sequence_models))
        print('combination_model_key = {0}'.format(self.model_data.combination_model_key))
        print('output_model_key = {0}'.format(self.model_data.output_model_key))
