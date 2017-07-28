#!/usr/bin/env python  

import rospy 
import tf
from geometry_msgs.msg import PoseStamped

map_frame = 'map'
marker_frame = 'ar_marker_'
cam1frame = 'stingray1'
cam2frame = 'stingray2'
MIN_MARKER = 1
MAX_MARKER = 4
MAX_BLACKOUT = .2

if __name__ == "__main__":
  # Initialize TF listener
  rospy.init_node('wall_publisher')

  listener = tf.TransformListener(10)

  # Initialize cmd_vel publisher
  pose_pub = rospy.Publisher('/walls', PoseStamped, queue_size=20)

  # Control loop
  period = 1.0
  rate = rospy.Rate(period)
  while not rospy.is_shutdown():
   
    for m in range(MIN_MARKER, MAX_MARKER + 1):
      frame_to_use = None
      marker_frame_full = marker_frame + str(m)
      now = rospy.Time.now() - rospy.Duration(period)
      
      #print "Checking " + cam1frame + "/" + marker_frame_full
      if listener.canTransform(map_frame, cam1frame + "/" + marker_frame_full, now):
        frame_to_use = cam1frame
      elif listener.canTransform(map_frame, cam2frame + "/" + marker_frame_full, now):
        frame_to_use = cam2frame
    
      if frame_to_use != None:
        (trans,rot) = listener.lookupTransform(map_frame, frame_to_use + '/' + marker_frame_full, now)

        # Send the transform as a msg too for java nodes - 2d
        p = PoseStamped()
        p.pose.position.x = trans[0]
        p.pose.position.y = trans[1]
        euler = tf.transformations.euler_from_quaternion(rot)
        # Cheat - send theta directly in w
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 1
        p.pose.orientation.w = euler[2]
        p.header.frame_id = str(m) 
        pose_pub.publish(p)
            
    rate.sleep()

  
