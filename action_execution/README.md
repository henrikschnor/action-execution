# action_execution

## Summary

The module contains:
* An action execution ontology for defining actions, action execution models, and known failure cases for those actions
* A set of Python scripts and packages that use the ontology together with an implementation of the execution models for obtaining execution data
* A MongoDB-based execution data logger

There are three main use cases for this module:
1. Obtaining execution data based on some known semantic properties about a given scenario (e.g. releasing an object on a table, grasping an object from a box, and so forth)
2. Execution data analysis and visualisation
3. Continuous execution model learning based on the logged execution data; this however depends on the models themselves being learning-based

*Note*: This is work in progress; at the moment, the module contains a very small ontology and implementations of only a few execution models. Unit test scripts for the existing actions and models can be found under `tests`.

## Directory structure

The directory structure of `action_execution` is shown below:

```
action_execution
|    __init__.py
|    action.py
|    config_keys.py
|    execution_model.py
|    failure_case.py
|____execution_models
     |    __init__.py
     |    <model_1>.py
     |    ...
     |____<model_n>.py
     |
     extern
     |    __init__.py
     |____transformations.py
     |
     geometry
     |    __init__.py
     |    bbox.py
     |    object.py
     |    pose.py
     |____vector.py
     |
     logger
     |    __init.py__
     |____execution_data_logger.py
     |
     ontology
     |____actions
     |    |    <action_1>.yaml
     |    |    ....
     |    |____<action_n>.yaml
     |    |
     |____execution_models
     |    |    <model_1>.yaml
     |    |    ...
     |    |____<model_n>.yaml
     |    |
     |____failure_cases
     |    |    <failure_case_1>.yaml
     |    |    ...
     |    |____<failure_case_n>.yaml
     test_utils
     |    __init__.py
     |    <util_1>.py
     |    ...
     |____<util_n>.py
     |
     tests
     |____actions
     |    |    <action_1>_action_test.py
     |    |    ....
     |    |____<action_n>_action_test.py
     |    |
     |____execution_models
     |    |    <model_1>_model_test.py
     |    |    ...
     |    |____<model_n>_model_test.py
     |    |
     |____failure_cases
     |    |    <failure_case_1>_failure_test.py
     |    |    ...
     |    |____<failure_case_n>_failure_test.py
     utils
     |    __init__.py
     |____configuration.py

```

## Usage

There is currently no installation script for the package, so the following steps need to be taken to use the module:
1. Add the path of the module's parent directory to the `PYTHONPATH` environment variable:
```
export PYTHONPATH=<path-to-action-execution-repository>:$PYTHONPATH
```
2. In `config_keys.py`, assign the `DIR_ABS_PATH` variable - defined in `PathConfig` - to the location of this module

## Example usage

Examples of invoking the action execution interface and the individual execution models can be found in the `tests` subdirectory. ROS-based examples of visualising the collected execution data for debugging the execution models can be found [here](../action_execution_ros).
