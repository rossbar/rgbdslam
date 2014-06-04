#! /usr/bin/env python
import numpy as np
import os
import time

import roslib; roslib.load_manifest('rgbdslam')
import rospy
import rgbdslam.srv as srv

class Controller():
  '''Class for controlling the start/stopping of streaming kinect data and
     saving of point clouds by the rgbdslam algorithm via ros service proxies'''
  def __init__( self ):
    self.ros_ui = rospy.ServiceProxy( '/rgbdslam/ros_ui',\
                                      srv.rgbdslam_ros_ui )
    self.ros_ui_b = rospy.ServiceProxy( '/rgbdslam/ros_ui_b',\
                                        srv.rgbdslam_ros_ui_b )

  def start( self ):
    rospy.wait_for_service('/rgbdslam/ros_ui_b')
    self.ros_ui_b('pause', False)

  def stop( self ):
    rospy.wait_for_service('/rgbdslam/ros_ui_b')
    self.ros_ui_b('pause', True)

  def sendAggregateCloud( self ):
    rospy.wait_for_service('/rgbdslam/ros_ui_s')
    self.ros_ui('quick_save')

if __name__ == '__main__':
  rospy.init_node('rgbdslam_controller')
  control = Controller()
  # Test that it works
  raw_input('Press any key to start streaming/sending pointclouds...')
  start_time = rospy.get_time()
  control.start()
  print 'Controller started at %s' %(start_time)
  while not rospy.is_shutdown():
    save_start = rospy.get_time()
    print 'Saving cloud...'
    control.sendAggregateCloud()
    save_end = rospy.get_time()
    print 'Done. %s seconds to save cloud' %(save_end - save_start)
    time.sleep(10)
    
#  time.sleep(10)
#  stop_time = rospy.get_time()
#  control.stop()
#  print 'Controller stopped at %s. Elapsed up-time = %s' %(stop_time,\
#                                                         stop_time-start_time)
#  save_start = rospy.get_time()
#  print 'Saving cloud...'
#  control.sendAggregateCloud()
#  save_end = rospy.get_time()
#  print 'Done. %s seconds to save cloud' %(save_end - save_start)
#  rospy.init_node('controller')
#  ros_ui_b = rospy.ServiceProxy('/rgbdslam/ros_ui_b',\
#                                srv.rgbdslam_ros_ui_b)
#  rospy.wait_for_service('/rgbdslam/ros_ui_b')
#  ros_ui_b('pause', False)
