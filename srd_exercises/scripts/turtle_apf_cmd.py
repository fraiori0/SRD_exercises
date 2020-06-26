#!/usr/bin/env python  
import roslib
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
from math import *
import matplotlib.pyplot as plt
import numpy as np
from srd_exercises.msg import Vel_custom

class APF:
    def __init__(self,turtle_num,hz,k_rep,k_att):
        self.turtle_num = turtle_num
        self.hz = hz
        self.k_att = k_att
        self.k_rep = k_rep
        self.eta_0 = 2. # obstacle radius of influence
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
        self.sub_target = rospy.Subscriber('APF_target',Pose,callback=self.set_goal)
        self.rate = rospy.Rate(self.hz)
        self.null_speed = Vel_custom(lin=0.,ang=0.)
        self.apf_cmd = Vel_custom(lin=0.,ang=0.)
        self.pub_cmd = rospy.Publisher('turtle1_custom_cmd', Vel_custom, queue_size=20)
        self.goal = np.array((0.,0.))
        #self.e_goal = np.array((1.,1.))
        rospy.sleep(0.1)
    def update(self, msg, turtle_id):
        # update the position of the turtle that moved
        self.pose_list[turtle_id-1] = msg
    def set_goal(self,msg):
        self.goal = np.array((msg.x,msg.y))
        rospy.loginfo('Goal set at (%f,%f)' % (msg.x,msg.y))
        for i in range(1,turtle_num):
            rospy.loginfo('Obstacle'+str(i) + ' in (%s,%s)' %(self.pose_list[i].x, self.pose_list[i].y))
        # start moving
        self.e_goal=1.
        while ((np.linalg.norm(self.e_goal) > 0.1) and rospy.ROSInterruptException):
            self.move_turtle()
            self.rate.sleep()
        self.pub_cmd.publish(controller.null_speed)
    def attractive_gradient(self,x,y):
        self.e_goal = self.goal - np.array((x,y))
        return self.k_att * self.e_goal
    def repulsive_gradient(self,x,y,x_obs,y_obs):
        # obstacle radius of influence eta_0 = 0.5m 
        pos = np.array((x,y))
        obs_pos = np.array((x_obs,y_obs))
        d = np.linalg.norm(pos-obs_pos)
        if d < self.eta_0:
            v = (pos-obs_pos)
            return v * self.k_rep * (1./d - 1./self.eta_0)/ (d**(3./2.))
        else:
            return np.array((0.,0.))
    def move_turtle(self):
        curr_pos = np.array((self.pose_list[0].x,self.pose_list[0].y))
        grad = self.attractive_gradient(curr_pos[0],curr_pos[1])
        for i in range(1,turtle_num):
            grad = grad + self.repulsive_gradient(curr_pos[0],curr_pos[1],self.pose_list[i].x,self.pose_list[i].y)
        n_g = np.linalg.norm(grad)
        th_g = atan2(grad[1],grad[0])
        e_o = th_g - self.pose_list[0].theta
        self.apf_cmd.lin = 0.4*(n_g * cos(e_o))
        self.apf_cmd.ang = 10*e_o
        self.pub_cmd.publish(self.apf_cmd)
    def plot_gradient(self):
        x_grid, y_grid = np.meshgrid(np.linspace(0,11,45), np.linspace(0,11,45))
        point_array = np.transpose(np.vstack((np.ndarray.flatten(x_grid),np.ndarray.flatten(y_grid))))
        grad_array = np.zeros(point_array.shape)
        for i in range(point_array.shape[0]):
            grad = self.attractive_gradient(point_array[i,0],point_array[i,1])
            for j in range(1,turtle_num):
                grad = grad + self.repulsive_gradient(point_array[i,0],point_array[i,1],self.pose_list[j].x,self.pose_list[j].y)
            grad_array[i] = grad
        fig,axs = plt.subplots(figsize=(7,7))
        axs.quiver(
            point_array[:,0],
            point_array[:,1],
            grad_array[:,0],
            grad_array[:,1],
            np.hypot(grad_array[:,0],grad_array[:,1]), #color map
            units='width')
        axs.set(title='APF gradient',xlabel='x [m]',ylabel='y [m]')
        plt.savefig('/home/fra/Uni/Ing Robotica/Sistemi Robotici Distribuiti/fig/gradient_barrier.png')




if __name__ == '__main__':
    try:
        rospy.init_node('APF_generator',anonymous=True)
        turtle_num = rospy.get_param('~turtle_num')
        hz = rospy.get_param('~hz',default=50)
        k_rep = rospy.get_param('~k_rep',default=4.)
        k_att = rospy.get_param('~k_att',default=1.)
        controller = APF(turtle_num,hz,k_rep=k_rep,k_att=k_att)
        # controller.plot_gradient()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
