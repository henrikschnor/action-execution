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

from os.path import join
import time
from importlib import import_module

from mas_knowledge_utils.ontology_query_interface import OntologyQueryInterface

from action_execution.config_keys import PathConfig, ActionConfigKeys, \
                                         LoggerConfigKeys, OBJ_MODULE_MAPPING
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
        self.compose_models = False
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
        self.ontology = OntologyQueryInterface(PathConfig.ONTOLOGY_URL,
                                               PathConfig.ONTOLOGY_ENTITY_PREFIX)
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
                if self.model_data.compose_models and model_results:
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
        # we get all ancestor actions of the current action and load the
        # configuration of each ancestor action
        self.action_data.id = action_name
        self.action_data.instance_of = self.ontology.get_objects_of(ActionConfigKeys.INSTANCE_OF,
                                                                    action_name)

        for action in self.action_data.instance_of:
            self.__load_parent_action_config(action)

        # we get the required capabilities of the action and add them to the list
        # of capabilities (but make sure there are no duplicates)
        capabilities = self.ontology.get_objects_of(ActionConfigKeys.CAPABILITIES, action_name)
        self.action_data.capabilities += [c for c in capabilities
                                          if c not in self.action_data.capabilities]

        # we get the failure cases of the action and add them to the list
        # of failure cases (but make sure there are no duplicates)
        failure_cases = self.ontology.get_objects_of(ActionConfigKeys.FAILURE_CASES, action_name)
        self.failure_case_data.keys += [f for f in failure_cases
                                        if f not in self.failure_case_data.keys]

        # we get the execution models of the action and add them to the list
        # of models (but make sure there are no duplicates)
        models = self.ontology.get_objects_of(ActionConfigKeys.MODELS, action_name)
        self.model_data.keys += [m for m in models if m not in self.model_data.keys]

        # we check whether the models need to be sequenced
        compose_models = self.ontology.get_objects_of(ActionConfigKeys.MODELS, action_name)
        if compose_models:
            self.model_data.compose_models = True

        # we get the combination model (if defined)
        comb_model = self.ontology.get_objects_of(ActionConfigKeys.COMBINATION_MODEL, action_name)
        if comb_model:
            self.model_data.combination_model_key = comb_model[0]

        # we get the output model (if defined)
        output_model = self.ontology.get_objects_of(ActionConfigKeys.OUTPUT_MODEL, action_name)
        if output_model:
            self.model_data.output_model_key = output_model[0]

        # for each model key, we create an instance of the
        # model config class and the model
        for model_key in self.model_data.keys:
            self.model_data.config.append(ExecutionModel(model_key))

            model_module = 'action_execution.execution_models.' + model_key
            model_class = getattr(import_module(model_module), model_key)
            self.model_data.models.append(model_class)

        if self.model_data.combination_model_key is not None:
            model_key = self.model_data.combination_model_key
            model_module = 'action_execution.execution_models.' + model_key
            model_class = getattr(import_module(model_module), model_key)
            self.model_data.combination_model = model_class

        if self.model_data.output_model_key is not None:
            model_key = self.model_data.output_model_key
            model_module = 'action_execution.execution_models.' + model_key
            model_class = getattr(import_module(model_module), model_key)
            self.model_data.output_model = model_class

        for failure_case in self.failure_case_data.keys:
            self.failure_case_data.config.append(FailureCase(failure_case))

        print('Description of "{0}" loaded successfully'.format(self.action_data.id))

    def __load_parent_action_config(self, action_name):
        '''Loads the config files of the parents of the current action
        and adds their capabilities, known failure cases, and execution models
        to the current action's respective lists

        Keyword arguments:
        @param action_name -- name of an action

        '''

        capabilities = self.ontology.get_objects_of(ActionConfigKeys.CAPABILITIES, action_name)
        for capability in capabilities:
            if capability not in self.action_data.capabilities:
                self.action_data.capabilities.insert(0, capability)

        failure_cases = self.ontology.get_objects_of(ActionConfigKeys.FAILURE_CASES, action_name)
        for failure in failure_cases:
            if failure not in self.failure_case_data.keys:
                self.failure_case_data.keys.insert(0, failure)

        models = self.ontology.get_objects_of(ActionConfigKeys.MODELS, action_name)
        for model in models:
            if model not in self.model_data.keys:
                self.model_data.keys.insert(0, model)

        compose_models = self.ontology.get_objects_of(ActionConfigKeys.MODELS, action_name)
        if compose_models:
            self.model_data.compose_models = True

        comb_model = self.ontology.get_objects_of(ActionConfigKeys.COMBINATION_MODEL, action_name)
        if comb_model:
            self.model_data.combination_model_key = comb_model[0]

        output_model = self.ontology.get_objects_of(ActionConfigKeys.OUTPUT_MODEL, action_name)
        if output_model:
            self.model_data.output_model_key = output_model[0]

    def print_config(self):
        '''Prints the values of the action's config fields
        '''
        print('action_id = {0}'.format(self.action_data.id))
        print('instance_of = {0}'.format(self.action_data.instance_of))
        print('capabilities = {0}'.format(self.action_data.capabilities))
        print('failure_case_keys = {0}'.format(self.failure_case_data.keys))
        print('model_keys = {0}'.format(self.model_data.keys))
        print('compose_models = {0}'.format(self.model_data.compose_models))
        print('combination_model_key = {0}'.format(self.model_data.combination_model_key))
        print('output_model_key = {0}'.format(self.model_data.output_model_key))
