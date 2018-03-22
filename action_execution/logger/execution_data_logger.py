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

class ExecutionDataLogger(object):
    def __init__(self, db_name):
        self.db_name = db_name
        mongo_client = pm.MongoClient()
        self.db = mongo_client[self.db_name]

    def log_metadata(self, collection_name, metadata, doc_id=None):
        collection = self.db[collection_name]
        document = metadata
        document['model_data'] = dict()
        result = collection.insert_one(document)
        doc_id = result.inserted_id
        return doc_id

    def log_model_data(self, collection_name, document_data, doc_id):
        collection = self.db[collection_name]
        document = collection.find_one({'_id': doc_id})
        document['model_data'].update(document_data)
        collection.replace_one({'_id': doc_id}, document)
        return doc_id

    def get_action_data(self, collection_name, action_name,
                        start_timestamp, end_timestamp):
        collection = self.db[collection_name]

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
