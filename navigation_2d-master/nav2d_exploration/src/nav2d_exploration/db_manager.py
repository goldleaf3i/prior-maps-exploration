from pymongo import MongoClient
from pymongo.errors import ConnectionFailure
import rospy
from std_msgs.msg import String
from nav2d_exploration.msg import Log

        
class DBManager:

    def __init__(self):
        self.connected = False
        
    def connectToDB(self):
        client = MongoClient("localhost", 27017)
        try:
            client.admin.command('isMaster')
            rospy.logdebug("[DBManager] Connected to mongoDB")
            self.connected = True
            self.db = client.blueprintLogXPS_CLOSEDOOR
            length = len(self.db.collection_names()) + 1
            collectionName = "run" + str(length)
            self.collection = self.db.create_collection(collectionName)
        except ConnectionFailure:
            self.connected = False
            rospy.logerr("[DBManager] Error in connecting to MongoDB")
            
    def isConnected(self):
        return self.connected
    
    def disconnectFromDB(self):
        self.client.close()
        self.isConnected = False
        
    def saveToDB(self, log):
        self.collection.insert_one({'Time': log.time, 'TravelledDistance': log.travelled_distance, 'Alpha': log.alpha, 'FrontierDistance': log.frontier_distance,
                                    'FrontierInfoGainFloorplan': log.frontier_information_gain_floorplan, 'FrontierInfoGainNoFloorplan': log.frontier_information_gain_no_floorplan,
                                     'FrontierUtility': log.frontier_utility, 'FrontierInfoGainMode': log.frontier_information_gain_mode,
                                     'CoveragePercentage': log.coverage_percentage});
            