#!/usr/bin/env python  
import roslib
import rospy
import tf
import turtlesim.msg
from visualization_msgs.msg import Marker

def handle_turtle_pose(msg, turtlename):
    quat = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     quat,
                     rospy.Time.now(),
                     turtlename,
                     "map")
if __name__ == '__main__':
    # initialize node and rviz publisher
    rospy.init_node('turtle_tf_broadcaster', anonymous=True)
    turtlename = rospy.get_param('~turtle', 'turtle1')
    # subscribe to turtlename/pose
    rospy.Subscriber(
        '/%s/pose' % turtlename,
        turtlesim.msg.Pose,
        handle_turtle_pose,
        turtlename
        )
    rospy.spin()