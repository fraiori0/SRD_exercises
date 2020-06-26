#!/usr/bin/env python  
import roslib
import rospy
from numpy.random import uniform
from srd_exercises.msg import Vel_custom
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from math import *

class Turtle_Control_Publisher:
    def __init__(self,turtlename,x0,y0,hz):
        self.turtlename = turtlename
        self.x0 = x0
        self.y0 = y0
        self.hz = hz
        # check if the initial value set for y0 is correct
        if (self.y0 < 5.5) or (self.y0 >10.):
            rospy.logwarn('y0 must be set between (5., 10.)')
            self.y0 = 5.5
        self.pub = rospy.Publisher('%s_custom_cmd' % self.turtlename, Vel_custom, queue_size=20)
        rospy.wait_for_service('%s/teleport_absolute' % self.turtlename)
        try:
            self.teleporter = rospy.ServiceProxy('%s/teleport_absolute' % self.turtlename, TeleportAbsolute)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        self.rate = rospy.Rate(self.hz)
        self.t = 1/self.hz
        self.pose = Pose()
        self.yp = 0.
        self.null_speed = Vel_custom(lin=0.,ang=0.)
        self.random_cmd = Vel_custom()
        self.sub = rospy.Subscriber('%s/pose' % self.turtlename,Pose,callback=self.update_pose)
    def update_pose(self,msg):
        self.pose = msg
    def gen_cmd(self):
        rospy.loginfo('actual y: %f' % controller.pose.y)
        rospy.loginfo('predicted y: %f' % self.yp)
        self.random_cmd.lin = uniform(-3.0,3.0)
        self.random_cmd.ang = uniform(-3.0,3.0)
        v = self.random_cmd.lin
        w = self.random_cmd.ang
        # predict y if moving from the current position
        # applying both v_lin e w_ang for t = 1/hz
        self.yp = self.pose.y - v/w*(cos(self.pose.theta + w*self.t)-cos(self.pose.theta))
        while self.yp < 5.5 :
            # repeat until it finds a combination that satisfies the constraint
            self.random_cmd.lin = uniform(-3.0,3.0)
            self.random_cmd.ang = uniform(-3.0,3.0)
            v = self.random_cmd.lin
            w = self.random_cmd.ang
            self.yp = self.pose.y - v/w*(cos(self.pose.theta + w*self.t)-cos(self.pose.theta))
    def teleport(self,x0,y0):
        self.teleporter(x0,y0,0.)

if __name__ == '__main__':
    try:
        # generate class instance
        rospy.init_node('turtle_random_cmd', anonymous=True)
        #
        turtlename = rospy.get_param('~turtlename', default='turtle1')
        x0 = rospy.get_param('~x0', default=5.5)
        y0 = rospy.get_param('~y0', default=5.5)
        hz = rospy.get_param('~hz', default=1)
        controller = Turtle_Control_Publisher(turtlename,x0,y0,hz)
        controller.teleport(controller.x0,controller.y0)
        while not rospy.is_shutdown():
            # first stop the turtle
            controller.pub.publish(controller.null_speed)
            # then compute velocity command
            controller.gen_cmd()
            controller.pub.publish(controller.random_cmd)
            controller.rate.sleep()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
