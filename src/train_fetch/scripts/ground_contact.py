#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ContactsState

d = []


def contactMade(data):
    states = data.states
    for state in states:
        if state.collision1_name.startswith("fetch"):
            name = state.collision1_name
            if "wheel" not in name and "base_link" not in name:
                print((state.collision1_name, state.collision2_name))


namespace = ""
rospy.init_node("GroundTest", anonymous=False)
rospy.Subscriber(namespace + "/ground_contact_topic", ContactsState, contactMade)

rospy.spin()
print(d)
