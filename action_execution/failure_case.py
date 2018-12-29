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

from action_execution.config_keys import PathConfig, FailureCaseConfigKeys

class Predicate(object):
    def __init__(self, name=None, args=list(), negated=False):
        self.name = name
        self.args = list(args)
        self.negated = negated

    def print_config(self):
        '''Prints the predicate's name, arguments,
        and whether the predicate is negated
        '''
        print('        name = {0}'.format(self.name))
        print('        args = {0}'.format(self.args))
        print('        negated = {0}'.format(self.negated))

class FailureCaseDefinition(object):
    def __init__(self, quantifier=None, variables=list(),
                 variable_types=list(), predicates=list()):
        self.quantifier = quantifier
        self.variables = list(variables)
        self.variable_types = list(variable_types)
        self.predicates = list(predicates)

    def print_config(self):
        '''Prints the definition of the failure case
        '''
        print('    quantifier = {0}'.format(self.quantifier))
        print('    variables = {0}'.format(self.variables))
        print('    variable_types = {0}'.format(self.variable_types))
        print('    predicates = ')
        for predicate in self.predicates:
            predicate.print_config()
            print()

class FailureCase(object):
    '''A description of a failure case that can be used for detecting
    the failure; this should then make it possible to take recovery actions.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, failure_case_name=None):
        self.failure_case_config_path = join(PathConfig.DIR_ABS_PATH,
                                             PathConfig.FAILURE_CASE_CONFIG_PATH)

        self.id = failure_case_name
        self.definition = FailureCaseDefinition()
        self.recovery_actions = list()

        if self.id is not None:
            self.load_config(self.id)

    def load_config(self, failure_case_name):
        '''Loads the config file of the failure case specified by 'failure_case_name'

        Keyword arguments:
        failure_case_name -- id of a failure case

        '''
        config_path = join(self.failure_case_config_path, failure_case_name + '.yaml')

        try:
            file_handle = open(config_path, 'r')
            config = yaml.load(file_handle)
            file_handle.close()

            self.id = failure_case_name
            if FailureCaseConfigKeys.DEFINITION in config:
                self.__load_definition(config[FailureCaseConfigKeys.DEFINITION])
            if FailureCaseConfigKeys.RECOVERY_ACTIONS in config:
                self.recovery_actions = config[FailureCaseConfigKeys.RECOVERY_ACTIONS]

            print('Description of "{0}" loaded successfully'.format(self.id))
        except IOError:
            print('Cannot load config of unknown failure case "{0}"'.format(self.id))

    def __load_definition(self, config):
        '''Loads a first-order logic definition of a failure case

        Keyword arguments:
        config -- a dictionary specifying a first-order logic definition of a failure case

        '''
        self.definition.quantifier = config[FailureCaseConfigKeys.QUANTIFIER]
        self.definition.variables = config[FailureCaseConfigKeys.VARIABLES]
        self.definition.variable_types = config[FailureCaseConfigKeys.VARIABLE_TYPES]
        for pred in config[FailureCaseConfigKeys.PREDICATES]:
            predicate_config = pred[FailureCaseConfigKeys.PREDICATE]

            predicate = Predicate()
            predicate.name = predicate_config[FailureCaseConfigKeys.PREDICATE_NAME]
            predicate.args = predicate_config[FailureCaseConfigKeys.PREDICATE_ARGS]
            predicate.negated = bool(predicate_config[FailureCaseConfigKeys.PREDICATE_NEGATED])
            self.definition.predicates.append(predicate)

    def print_config(self):
        '''Prints the values of the failure case's config fields
        '''
        print('failure_case_id = {0}'.format(self.id))
        print('definition = ')
        self.definition.print_config()
        print('recovery_actions = {0}'.format(self.recovery_actions))
