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
from os.path import join
import yaml

from action_execution.config_keys import PathConfig, ModelConfigKeys

class ExecutionModel(object):
    '''A description of an action execution model.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, model_name=None):
        self.config_path = join(PathConfig.DIR_ABS_PATH,
                                PathConfig.MODEL_CONFIG_PATH)

        self.id = model_name
        self.inputs = dict()
        self.outputs = dict()
        self.params = dict()

        if self.id is not None:
            self.load_config(self.id)

    def load_config(self, model_name):
        '''Loads the config file of the model specified by 'model_name'

        Keyword arguments:
        model_name -- id of a model

        '''
        config_path = join(self.config_path, model_name + '.yaml')

        try:
            file_handle = file(config_path, 'r')
            config = yaml.load(file_handle)
            file_handle.close()

            self.id = model_name
            if ModelConfigKeys.INPUTS in config:
                for opt_in in config[ModelConfigKeys.INPUTS]:
                    input_config = opt_in[ModelConfigKeys.INPUT]
                    input_name = input_config[ModelConfigKeys.INPUT_NAME]
                    input_type = input_config[ModelConfigKeys.INPUT_TYPE]
                    self.inputs[input_name] = input_type
            if ModelConfigKeys.OUTPUTS in config:
                for opt_out in config[ModelConfigKeys.OUTPUTS]:
                    output_config = opt_out[ModelConfigKeys.OUTPUT]
                    output_name = output_config[ModelConfigKeys.OUTPUT_NAME]
                    output_type = output_config[ModelConfigKeys.OUTPUT_TYPE]
                    self.outputs[output_name] = output_type
            if ModelConfigKeys.PARAMS in config:
                for opt_params in config[ModelConfigKeys.PARAMS]:
                    param_config = opt_params[ModelConfigKeys.PARAM]
                    param_name = param_config[ModelConfigKeys.PARAM_NAME]
                    param_type = param_config[ModelConfigKeys.PARAM_TYPE]
                    self.params[param_name] = param_type

            print('Description of "{0}" loaded successfully'.format(self.id))
        except IOError:
            print('Cannot load config of unknown optimiser "{0}"'.format(self.id))

    def verify_input(self, argument_dict):
        '''Checks whether the input in kwargs corresponds to the
        input description specified in the config. The input is considered
        invalid if it contains:
            * an unexpected key or
            * an object of an incorrect type for a given key (including
              a list of incorrect objects)

        Returns:
        valid_input -- a Boolean specifing whether the input is valid
        message -- a string with error messages about the input;
                   empty string if the input is valid

        '''
        valid_input = True
        message = ''
        for key, value in argument_dict.iteritems():
            # the input is invalid if a list of a given type is expected
            # for a given key, but not passed, or if an object of an incorrect
            # type is passed
            if key in self.inputs:
                list_expected = self.inputs[key].find('[]') != -1
                if list_expected:
                    valid_list, new_message = self.__verify_list_input(key, value)
                    if valid_input:
                        valid_input = valid_list
                    message += new_message
                else:
                    valid_obj, new_message = self.__verify_object_input(key, value)
                    if valid_input:
                        valid_input = valid_obj
                    message += new_message
        return valid_input, message

    def __verify_object_input(self, key, value):
        '''Checks whether 'value' has the expected object type for 'key' as
        specified in the model description.

        valid_input -- a Boolean specifing whether the input is valid
        message -- an error message about the input;
                   empty string if the input is valid

        '''
        valid_input = True
        message = ''
        exp_input_type = self.inputs[key]
        content_type = type(value).__name__
        if content_type != exp_input_type:
            message = 'Expected input "' + exp_input_type \
                      + '" for key "' + key + '"'
            valid_input = False
        return valid_input, message

    def __verify_list_input(self, key, value):
        '''Checks whether 'value' is of type list and whether the list's
        objects are of the expected type for 'key' as specified
        in the model description.

        valid_input -- a Boolean specifing whether the input is valid
        message -- an error message about the input;
                   empty string if the input is valid

        '''
        valid_input = True
        message = ''
        if type(value).__name__ != 'list':
            message = 'Expected input "list" for key "' + key + '"'
            valid_input = False
        else:
            bracket_idx = self.inputs[key].find('[]')
            exp_input_type = self.inputs[key][0:bracket_idx]
            if value:
                list_content_type = type(value[0]).__name__
                if list_content_type != exp_input_type:
                    message = 'Expected list of "' + exp_input_type \
                              + '" for key "' + key + '"'
                    valid_input = False
        return valid_input, message

    def print_config(self):
        '''Prints the values of the model's config fields
        '''
        print('model_id = {0}'.format(self.id))
        print('Inputs:')
        for key, value in self.inputs.iteritems():
            print('    {0}: {1}'.format(key, value))
        print('Outputs:')
        for key, value in self.outputs.iteritems():
            print('    {0}: {1}'.format(key, value))
