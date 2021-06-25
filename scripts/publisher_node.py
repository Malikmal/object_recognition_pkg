#!/usr/bin/env python

import rospy 
from std_msgs.msg import String
from object_recognition_pkg.msg import data_completed

def talker():
    # pub = rospy.Publisher('chatter',  String, queue_size=10)
    pub = rospy.Publisher('chatter',  data_completed)
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(10) #10 Hz

    str = ['laskdjalksjd','asd']

    msg = data_completed()
    msg.test = "hello"
    msg.test_array_string = str
    
    while not rospy.is_shutdown():
        hello_str = "Hello World %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        rospy.loginfo(msg.test)
        # pub.publish(hello_str)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__': 
    try:
        talker()
    except rospy.ROSInternalException:
        pass