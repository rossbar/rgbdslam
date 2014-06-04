#! /usr/bin/env python
import numpy as np
import os

import roslib; roslib.load_manifest('rgbdslam')
import rospy

from sensor_msgs.msg import PointCloud2
from python_msg_conversions import pointclouds

class PointCloudListener():
  '''Creates an object that listens on the rgbdslam/batchclouds topic for 
     point clouds.'''

  def __init__( self ):
    self.cloud_arr = []
    self.ctr = 0	# For down sampling
    print 'Initializing point cloud listener...'
    rospy.init_node('ptcloud_listener', anonymous=True)
    print 'Done.'
    print 'Subscribing to rgbdslam/aggregate_clouds...'
    rospy.Subscriber("rgbdslam/aggregate_clouds", PointCloud2, self.callback)
    print 'Done.'
    rospy.spin()

  def callback( self, cloud_msg ):
    '''Function that the subscriber calls when it gets a new message'''
  #  print 'callback fn from ptcloud_listener called'
  #  rospy.loginfo( rospy.get_name() + ": pointcloud rcvd from %s" \
  #                 %str(cloud_msg.PointField.name) )
  ##  rospy.loginfo( rospy.get_name() + ": pointcloud msg rcvd" )
  #  print 'rospy logging successful'
    if self.cloud_arr == []:
      self.cloud_arr = pointclouds.pointcloud2_to_array( cloud_msg,\
                                                         split_rgb=True )
      self.cloud_arr = self.cloud_arr[ np.isfinite( self.cloud_arr['z'] ) ]
    elif self.ctr % 10 == 0:
      cloud_arr = pointclouds.pointcloud2_to_array( cloud_msg, split_rgb=True )
      cloud_arr = cloud_arr[ np.isfinite( cloud_arr['z'] ) ]
      self.cloud_arr = np.concatenate( (self.cloud_arr, cloud_arr) )
    self.ctr += 1

#    if not os.path.exists( 'ptcloud.npy' ):
  #  print 'pointcloud conversion successful'
  #  print 'Frame id = ', str(cloud_msg.header.frame_id)
  #  if str( cloud_msg.header.frame_id ) == '1':
    print 'Saving ptcld...'
    np.save( '/home/grim4/Desktop/ptcloud.npy', self.cloud_arr )
    print 'Done.'
  
if __name__ == '__main__':
  l = PointCloudListener()
