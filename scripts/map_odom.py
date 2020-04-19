#!/usr/bin/env python  
import roslib
import rospy

import tf
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

def handle_turtle_pose(msg,args):
    br = tf.TransformBroadcaster()
    q = quaternion_from_euler(0, 0, float(args[2]))
    br.sendTransform((args[0],args[1], 0),
                     (q[0],q[1],q[2],q[3]),
                     rospy.Time.now(),
                     "/"+args[3]+"/odom",
                     "map")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    x = rospy.get_param('~x_co')
    y = rospy.get_param('~y_co')
    theta = rospy.get_param('~theta')
    robot_name = rospy.get_param('~robot_name')
    rospy.Subscriber('/'+robot_name+'/base_pose_ground_truth',
                     Odometry,
                     handle_turtle_pose,(x,y,theta,robot_name))
    rospy.spin()
