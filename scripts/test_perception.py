#!/usr/bin/env python3

import rospy
import rospkg
import rosbag
from sofia_perception.srv import *

def main():
    rospy.init_node('test_perception')
    detect_object_client = rospy.ServiceProxy('detect_object', DetectObject)

    rospack = rospkg.RosPack()
    bag_path = rospack.get_path('sofia_perception') + '/resources/perception.bag'
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=['/pc_glob']):
        res = detect_object_client.call(msg)
        print(res)
    bag.close()

if __name__ == "__main__":
    main()