#!/usr/bin/env python
from math import *

import rospy

import matplotlib.pyplot as plt

from scitos_common.msg import Vec2Array

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", data.x, data.y)
    plt.clf()
    plt.scatter([atan(e) for e in data.x], [e for e in data.y])
    #plt.xlim([-1e4, 1e4])
    #plt.ylim([-1e5, 1e5])

    plt.draw()
    plt.pause(0.00000000001)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hough_plotter', anonymous=True)

    rospy.Subscriber("/debug/map_line_hough", Vec2Array, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()