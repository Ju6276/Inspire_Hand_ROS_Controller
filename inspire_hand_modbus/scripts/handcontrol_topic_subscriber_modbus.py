#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String


def callback(msg):
    """
    callback function: process the received touch data
    """
    rospy.loginfo(f"received touch data:\n{msg.data}")


def main():
    rospy.init_node("handcontrol_subscriber", anonymous=True)

    # create the subscriber
    rospy.Subscriber("touch_data", String, callback)

    try:
        rospy.spin()  # keep the node running
    except rospy.ROSInterruptException:
        rospy.loginfo("touch subscriber node is stopped manually")


if __name__ == "__main__":
    main()

