#!/usr/bin/env python  
import roslib
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
from math import *

class Baricenter:
    def __init__(self,turtle_num):
        self.turtle_num = turtle_num
        self.pose_list = []
        self.turtle_id_list = []
        self.sub_list = []
        for i in range(turtle_num):
            self.pose_list.append(Pose())
            self.turtle_id_list.append('turtle'+str(i+1))
            self.sub_list.append(
                rospy.Subscriber(
                    '/turtle'+str(i+1)+'/pose',
                    Pose,
                    callback=self.update,
                    callback_args=(i+1)
                    )
                )
        self.bar_pose = Pose()
        self.pub_bar = rospy.Publisher('formation_baricenter', Pose, queue_size=10)
        self.pub_whose_turn = rospy.Publisher('formation_whose_turn',String,queue_size=10)
        self.sub_just_moved = rospy.Subscriber('formation_just_moved',String,callback=self.select_turtle)
        rospy.sleep(0.5)
        # start movement of the first turtle
        self.pub_whose_turn.publish('turtle1')
    def update(self, msg, turtle_id):
        # update the position of the turtle that moved
        self.pose_list[turtle_id-1] = msg
        # compute the new baricenter
        self.bar_pose.x = 0.
        self.bar_pose.y = 0.
        for pose in self.pose_list:
            self.bar_pose.x = self.bar_pose.x + pose.x/self.turtle_num
            self.bar_pose.y = self.bar_pose.y + pose.y/self.turtle_num
        #rospy.loginfo('Baricenter updated: (%f,%f)' %(self.bar_pose.x,self.bar_pose.y))
        # publsh the position
        self.pub_bar.publish(self.bar_pose)
    def select_turtle(self,msg):
        for i in range(self.turtle_num):
            if (msg.data == ('turtle'+str(i+1))):
                self.pub_whose_turn.publish('turtle'+str(((i+1) % self.turtle_num)+1))
                rospy.loginfo('Now is %s turn' % msg.data)
                break

if __name__ == '__main__':
    try:
        rospy.init_node('baricenter_computation',anonymous=True)
        turtle_num = rospy.get_param('~turtle_num')
        bar = Baricenter(turtle_num)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()