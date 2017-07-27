#!/usr/bin/env python  

import rospy 
import tf
from geometry_msgs.msg import Pose2D
from fiducial_msgs.msg import FiducialTransformArray

map_frame = 'map'
marker_frame = 'ar_marker_0'
cam1frame = 'stingray1'
cam2frame = 'stingray2'
MAX_BLACKOUT = .2

listener = None

def fiducial_callback(msg):
  now = rospy.Time.now()
  cam_transform = listener.lookupTransform(map_frame, cam1frame, now)
  for t in msg.transforms:
    

if __name__ == "__main__":
  # Initialize TF listener
  rospy.init_node('hover')

  listener = tf.TransformListener(1)
  tf_pub = tf.TransformBroadcaster()

  # Initialize cmd_vel publisher
  pose_pub = rospy.Publisher('/robotito/pose', Pose2D, queue_size=1)
  pose_listener = rospy.Subscriber("/stingray1/fiducial_transforms", FiducialTransformArray, fiducial_callback)

  rospy.spin()

  
