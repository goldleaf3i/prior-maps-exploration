#! /usr/bin/env python
from nav2d_exploration.db_manager import DBManager
from nav2d_exploration.msg import Log
import rospy
from std_msgs.msg import String

dbManager = DBManager()

def callback(data):
    global dbManager
    dbManager.saveToDB(data)

if __name__ == '__main__':
    try:
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('logger', anonymous=True)
        
        rospy.Subscriber("Navigator/log", Log, callback)

        dbManager.connectToDB()
        if (dbManager.isConnected()):
            # spin() simply keeps python from exiting until this node is stopped
            rospy.spin()
            
    except rospy.ROSInterruptException:
        pass