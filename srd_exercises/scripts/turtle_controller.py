#!/usr/bin/env python  
import roslib
import rospy
import tf
from math import *
from geometry_msgs.msg import Twist
from srd_exercises.msg import Vel_custom

class Turtle_Controller:
    def __init__(self):
        # accept as input the saturation parameters and the name of the turtle to be controlled
        rospy.init_node('turtle_controller', anonymous=True)
        self.sat_v = rospy.get_param('~sat_v', default=2.0)
        self.sat_w = rospy.get_param('~sat_w', default=2.0)
        self.turtlename = rospy.get_param('~turtlename', default='turtle1')
        self.cmd = Twist()
        self.sub = rospy.Subscriber('%s_custom_cmd' % self.turtlename, Vel_custom, self.callback)
        self.pub = rospy.Publisher('%s/cmd_vel' % self.turtlename, Twist, queue_size=20)
    def callback(self, msg):
        self.cmd.linear.x = copysign(1.0, msg.lin)*min(abs(msg.lin),self.sat_v)
        self.cmd.angular.z = copysign(1.0, msg.ang)*min(abs(msg.ang),self.sat_w)
        self.pub.publish(self.cmd)
        #print('published: [%f, %f]' % (self.cmd.linear.x, self.cmd.angular.z))

if __name__ == '__main__':
    try:
        controller = Turtle_Controller()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
