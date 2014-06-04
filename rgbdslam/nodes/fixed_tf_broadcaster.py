#!/usr/bin/env python
import numpy as np
import roslib
roslib.load_manifest('rgbdslam')

import rospy
import tf

def convertToQuaternion( R ):
  '''Convert orthonormal rotation matrix to quaternion representation.'''
  tr = np.trace( R )
  if tr > 0:
    S = 2 * np.sqrt( tr + 1.0 )
    qw = 0.25 * S
    qx = ( R[2,1] - R[1,2] ) / S
    qy = ( R[0,2] - R[2,0] ) / S
    qz = ( R[1,0] - R[0,1] ) / S
  elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
    S = 2 * np.sqrt( 1.0 + m00 - m11 - m22 )
    qw = ( R[2,1] - R[1,2] ) / S
    qx = 0.25 * S
    qy = ( R[0,1] + R[1,0] ) / S
    qz = ( R[0,2] + R[2,0] ) / S
  elif ( R[1,1] > R[2,2] ):
    S = 2 * np.sqrt( 1.0 + R[1,1] - R[0,0] - R[2,2] )
    qw = ( R[0,2] - R[2,0] ) / S
    qx = ( R[0,1] + R[1,0] ) / S
    qy = 0.25 * S
    qz = ( R[1,2] + R[2,1] ) / S
  else:
    S = 2 * np.sqrt( 1.0 + R[2,2] - R[0,0] - R[1,1] )
    qw = ( R[1,0] - R[0,1] ) / S
    qx = ( R[0,2] + R[2,0] ) / S
    qy = ( R[1,2] + R[2,1] ) / S
    qz = 0.25 * S
  return np.array( [qw, qx, qy, qz] )

if __name__ == '__main__':
    # Load transformation data
    fpath = '/home/grim4/PythonFramework/kinectCalibration/RT_kinect_to_detector.transform'
    RT = np.loadtxt( fpath )
    R = RT[:,:-1]
    T = RT[:,-1]
    # Convert rotation matrix to quaternion
    R4 = np.identity(4)
    R4[0:3,0:3] = R
    Q = tf.transformations.quaternion_from_matrix( R4 )
        
    rospy.init_node('kinect_to_detector_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(tuple(T),
                         tuple(Q),
                         rospy.Time.now(),
                         "detector_frame",
                         "camera_link")
        rate.sleep()
