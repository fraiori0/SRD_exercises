#!/usr/bin/env python  
import roslib
import rospy
from numpy.random import uniform
from srd_exercises.msg import Vel_custom
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from math import *
from std_msgs.msg import String

class Turtle_Control_Publisher:
    def __init__(self):
        #
        rospy.init_node('turtle_random_cmd', anonymous=True)
        #
        self.turtlename = rospy.get_param('~turtlename', default='turtle1')
        self.hz = rospy.get_param('~hz', default=50)
        # check if the initial value set for y0 is correct
        self.pub_cmd = rospy.Publisher('%s_custom_cmd' % self.turtlename, Vel_custom, queue_size=20)
        self.pub_just_moved = rospy.Publisher('formation_just_moved', String, queue_size=20) 
        rospy.wait_for_service('%s/teleport_absolute' % self.turtlename)
        try:
            self.teleporter = rospy.ServiceProxy('%s/teleport_absolute' % self.turtlename, TeleportAbsolute)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        self.rate = rospy.Rate(self.hz)
        self.pose = Pose()
        self.bar_pose = Pose()
        self.null_speed = Vel_custom(lin=0.,ang=0.)
        self.cmd = Vel_custom()
        self.sub_pose = rospy.Subscriber('/%s/pose' % self.turtlename,Pose,callback=self.update_pose)
        self.sub_bar = rospy.Subscriber('formation_baricenter',Pose,callback=self.update_baricenter)
        self.sub_whose_turn = rospy.Subscriber('formation_whose_turn',String,callback=self.is_my_turn)
    def update_pose(self,msg):
        self.pose = msg
    def update_baricenter(self,msg):
        self.bar_pose = msg
    def teleport(self,x0,y0):
        self.teleporter(x0,y0,0.)
    def orient(self):
        self.bar_angle = atan2(self.bar_pose.y-self.pose.y, self.bar_pose.x-self.pose.x)
        e_o = 1
        while (abs(e_o)>0.01):
            e_o = self.pose.theta - self.bar_angle
            self.cmd.ang = -e_o 
            self.cmd.lin = 0.
            self.pub_cmd.publish(self.cmd)
            self.rate.sleep()
    def move_forward(self):
        e_p = 1.
        while (abs(e_p)>0.1):
            e_p = ((self.bar_pose.x-self.pose.x)**2.+(self.bar_pose.y-self.pose.y)**2.)**(1./2.)
            self.cmd.lin = e_p 
            self.cmd.ang = 0.
            self.pub_cmd.publish(self.cmd)
            self.rate.sleep()
    def is_my_turn(self,msg):
        if not (msg.data == self.turtlename):
            return
        rospy.loginfo(self.turtlename+': is my turn')
        self.orient()
        self.move_forward()
        self.pub_just_moved.publish(self.turtlename)
        rospy.loginfo(self.turtlename+': I just moved')
        # stop the turtle
        self.pub_cmd.publish(self.null_speed)

if __name__ == '__main__':
    try:
        # generate class instance
        controller = Turtle_Control_Publisher()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()