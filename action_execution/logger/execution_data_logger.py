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

import pymongo as pm
import zmq
import json
from action_execution.config_keys import LoggerConfigKeys, BlackBoxConfig

class ExecutionDataLogger(object):
    def __init__(self):
        if not BlackBoxConfig.ENABLED:
            self.db_name = LoggerConfigKeys.DB_NAME
            print('Black box disabled; logging data in local database {0}'.format(self.db_name))
        else:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.PUB)
            self.socket.bind("tcp://*:{0}".format(BlackBoxConfig.PORT))
            print('Logging data on black box')

    def log_model_data(self, action_name, document_data):
        if not BlackBoxConfig.ENABLED:
            mongo_client = pm.MongoClient()
            db = mongo_client[self.db_name]
            collection = db[action_name]
            collection.insert_one(document_data)
        else:
            json_data = json.dumps(document_data)
            self.socket.send_multipart([bytearray(action_name, 'utf8'),
                                        bytearray(json_data, 'utf8')])

    def get_action_data(self, collection_name, action_name,
                        start_timestamp, end_timestamp):
        mongo_client = pm.MongoClient()
        db = mongo_client[self.db_name]
        collection = db[collection_name]

        doc = None
        if start_timestamp > 0. and end_timestamp > 0. and start_timestamp < end_timestamp:
            doc = collection.find_one({'action_name': action_name,
                                       'timestamp': {'$gt': start_timestamp,
                                                     '$lt': end_timestamp}},
                                      sort=[('timestamp', -1)])
        else:
            doc = collection.find_one({'action_name': action_name}, sort=[('timestamp', -1)])

        if doc is not None:
            return doc['model_data']
        return doc
