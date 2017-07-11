#!/usr/bin/env python  

import rospy 
import tf
import geometry_msgs.msg

map_frame = 'map'
marker_frame = 'ar_marker_0'
MAX_BLACKOUT = .5

if __name__ == "__main__":
  # Initialize TF listener
  rospy.init_node('hover')

  listener = tf.TransformListener()

  # Initialize cmd_vel publisher
  pose_pub = rospy.Publisher('/robotito/pose', geometry_msgs.msg.Pose, queue_size=1)

  # Control loop
  rate = rospy.Rate(50.0)
  while not rospy.is_shutdown():
    try:
        # Get oldest timestamp for transform
        latest = listener.getLatestCommonTime(map_frame, marker_frame)
        if (rospy.Time() - latest < rospy.Duration(MAX_BLACKOUT)):
          # Obtain the transform
          (trans,rot) = listener.lookupTransform(map_frame, marker_frame, rospy.Time())
          
          p = Pose()
          p.x = trans[0]
          p.y = trans[1]
          euler = tf.transformations.euler_from_quaternion(rot)
          p.theta = euler[2]
           
          pose_pub.publis(p)
          
    except (tf.Exception):
        pass
    rate.sleep()

  
