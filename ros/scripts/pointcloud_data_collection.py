#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import String
import numpy as np
import tf
from time import sleep

class StateWrapper():
    def __init__(self):
        self.event_out_msg = ""
        
    def reset(self):
        self.event_out_msg = ""
        
    def send_event(self, in_publisher, out_topic, in_event, expected_msg):
        in_publisher.publish(in_event)
        sleep(0.1)
        rospy.Subscriber(out_topic, String, self.eventCallback)
        if self.event_out_msg == expected_msg:
            return True
        else:
            return False

    def send_and_wait_for_event(self, event_in_publisher, out_topic, in_event, expected_msg):
        event_in_publisher.publish(in_event)
        while self.event_out_msg != expected_msg:
            rospy.Subscriber(out_topic, String, self.eventCallback)
    
    def wait_for_event(self, event_out_topic, expected_msg):
        while self.event_out_msg != expected_msg:
            rospy.Subscriber(event_out_topic, String, self.eventCallback)

    def eventCallback(self, userdata):
        self.event_out_msg = userdata.data

if __name__ == "__main__":
    rospy.init_node('pointcloud_data_collection')
    print "pointcloud_data_collection' node is running ..."

    pub_event_in = rospy.Publisher('/tabletop_pointcloud_segmentation_ros/table_top_segmentation/event_in', String, queue_size=1)
    
    #in_topic = '/tabletop_pointcloud_segmentation_ros/table_top_segmentation/event_in'
    out_topic = '/tabletop_pointcloud_segmentation_ros/table_top_segmentation/event_out'
    
    rate = rospy.Rate(10)
    sw = StateWrapper()
    dataset_size = 0
    if not rospy.has_param('~dataset_size'):
        dataset_size = 20
        rospy.logwarn("Dataset size is set to 20")
    else:
        dataset_size = rospy.get_param("~dataset_size")
  
    collected_data = 0

    while not rospy.is_shutdown():
        str_msg = String()
        while collected_data < dataset_size:
            str_msg.data = "e_start"
            flag_data_saved = False
            while not flag_data_saved:
                if sw.send_event(pub_event_in, out_topic, str_msg, 'e_started'):
                    flag_data_saved = True
                    collected_data += 1
                    print "Got {} saved".format(collected_data)
            # this delay is needed to make sure the object already in a stable state
            sleep(1.75)             
                    
        rate.sleep()
        
    rospy.spin()
