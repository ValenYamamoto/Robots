#!/usr/bin/python3

import rospy
import datetime

rospy.init_node("time")

d = rospy.Duration(5.0)
rospy.sleep(d)
a = datetime.datetime.now()
begin = rospy.Time.now()
rospy.sleep(d)
now = rospy.Time.now()
b = datetime.datetime.now()

print(begin.to_sec(), begin.nsecs)
print(now.secs, now.nsecs)
duration = now - begin
print(duration.to_sec(), duration.nsecs)
c = b - a
print(c)
