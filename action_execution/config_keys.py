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

class PathConfig(object):
    '''A collection of constants specifying the paths
    of various config files.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    ONTOLOGY_URL = 'file:///home/alex/catkin_workspaces/robocup_ws/src/action-execution/action_execution/ontology/actions.owl'
    ONTOLOGY_ENTITY_PREFIX = 'action'
    DIR_ABS_PATH = '<absolute-path-of-action_execution>'
    ACTION_CONFIG_PATH = 'ontology/actions'
    FAILURE_CASE_CONFIG_PATH = 'ontology/failure_cases'
    MODEL_CONFIG_PATH = 'ontology/execution_models'

class ActionConfigKeys(object):
    '''A collection of constants specifying the keys
    in an action config file.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    ID = 'action_id'
    INSTANCE_OF = 'isInstanceOf'
    CAPABILITIES = 'requires'
    FAILURE_CASES = 'hasFailurecase'
    MODELS = 'hasModel'
    COMPOSE_MODELS = 'composeModels'
    COMBINATION_MODEL = 'combinationModel'
    OUTPUT_MODEL = 'outputModel'

class FailureCaseConfigKeys(object):
    '''A collection of constants specifying the keys
    in a failure case config file.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    ID = 'failure_case_id'
    DEFINITION = 'definition'
    RECOVERY_ACTIONS = 'recovery_actions'
    QUANTIFIER = 'quantifier'
    VARIABLES = 'variables'
    VARIABLE_TYPES = 'variable_types'
    PREDICATES = 'predicates'
    PREDICATE = 'predicate'
    PREDICATE_NAME = 'name'
    PREDICATE_ARGS = 'args'
    PREDICATE_NEGATED = 'negated'

class ModelConfigKeys(object):
    '''A collection of constants specifying the keys
    in an execution model config file.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    ID = 'model_id'
    INPUTS = 'inputs'
    INPUT = 'input'
    INPUT_NAME = 'name'
    INPUT_TYPE = 'type'
    OUTPUTS = 'outputs'
    OUTPUT = 'output'
    OUTPUT_NAME = 'name'
    OUTPUT_TYPE = 'type'
    PARAMS = 'params'
    PARAM = 'param'
    PARAM_NAME = 'name'
    PARAM_TYPE = 'type'

class ExecutionConfigKeys(object):
    '''A collection of constants specifying the
    possible named parameters for an execution model.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    FRAME_ID = 'frame_id'
    MANIPULATED_OBJECT = 'manipulated_object'
    OBJECTS_ON_SURFACE = 'objects_on_surface'
    SURFACE = 'surface'
    TARGET_OBJECT = 'target_object'
    ROBOT_POSE = 'robot_pose'
    ARM = 'arm'

class LoggerConfigKeys(object):
    '''A collection of constants specifying
    names of database collections used by a data logger.

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    DB_NAME = 'action_execution'
    MODEL_INPUT_COLLECTION = 'model_inputs'
    MODEL_RESULT_COLLECTION = 'model_results'

    DB_NAME_DEBUG = 'action_execution_debug'
    MODEL_INPUT_COLLECTION_DEBUG = 'model_inputs_debug'
    MODEL_RESULT_COLLECTION_DEBUG = 'model_results_debug'

class BlackBoxConfig(object):
    ENABLED = False
    PORT = 5680

OBJ_MODULE_MAPPING = {'Pose2': 'geometry.pose',
                      'Pose3': 'geometry.pose',
                      'Vector2': 'geometry.vector',
                      'Vector3': 'geometry.vector',
                      'Object3d': 'geometry.object',
                      'BBox3': 'geometry.bbox'}
