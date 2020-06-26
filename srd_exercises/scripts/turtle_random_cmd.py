#!/usr/bin/env python  
import roslib
import rospy
import tf
from numpy.random import uniform
from srd_exercises.msg import Vel_custom

class Turtle_Control_Publisher:
    def __init__(self):
        # accept as input the name of the turtle for which command requests are published
        self.turtlename = rospy.get_param('~turtlename', default='turtle1')
        rospy.init_node('%s_random_cmd' % self.turtlename)
        self.pub = rospy.Publisher('%s_custom_cmd' % self.turtlename, Vel_custom, queue_size=20)

if __name__ == '__main__':
    try:
        turtlename = rospy.get_param('~turtlename', default='turtle1')
        hz = rospy.get_param('~hz', default=10)
        rospy.init_node('%s_random_cmd' % turtlename)
        pub = rospy.Publisher('%s_custom_cmd' % turtlename, Vel_custom, queue_size=20)
        rate = rospy.Rate(hz)
        random_cmd = Vel_custom()
        while not rospy.is_shutdown():
            random_cmd.lin = uniform(-4.,4.)
            random_cmd.ang = uniform(-4.,4.)
            pub.publish(random_cmd)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
