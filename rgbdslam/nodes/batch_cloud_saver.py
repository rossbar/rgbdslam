#! /usr/bin/env python
import numpy as np
import os
import time

import roslib; roslib.load_manifest('rgbdslam')
import rospy

from sensor_msgs.msg import PointCloud2
from python_msg_conversions import pointclouds

import tf

# Imports for visualization testing
import PySide
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
DOTSIZE = 0.1

cldType = np.dtype({ 'names':['x', 'y', 'z', 'r', 'g', 'b', 'a'],\
                     'formats':[np.float32, np.float32, np.float32, np.float32,\
                               np.float32, np.float32, np.float32] } )


class BatchCloudListener():
  '''Creates a listener object that listens on /rgbdslam/batch_clouds topic
     for PointCloud2 messages. Takes the resulting message, extracts the
     timestamp, converts the data to an np.ndarray, and uses 
     tf.TransformListener to lookup the transform between /camera_link and 
     /map at the given timestamp. Applies the transform to every point in the
     cloud and publishes to a new topic.'''
  def __init__( self ):
    self.Subscriber = rospy.Subscriber("rgbdslam/batch_clouds", PointCloud2,\
                                       self.callback )
    self.ctr = 1	# Keep track of # of pt_clouds rcvd

#    # For visualization testing
#    self.app = QtGui.QApplication([])
#    self.widget = gl.GLViewWidget()
#    self.widget.show()
#    self.widget.setWindowTitle('Registered Point Clouds')
#    self.imgPts = np.array([[0,0,0]])
#    self.colors = np.array([[0,0,1,1]])
#    self.img = gl.GLScatterPlotItem( pos=self.imgPts,\
#               color=self.colors, size=DOTSIZE )
#    self.widget.addItem( self.img )

  def callback( self, cloud_msg ):
    '''Callback function for dealing with every new received message.'''
    # Get Timestamp of cloud
#    ts = cloud_msg.header.stamp
#    # Get transform for batch cloud at given time (NEED TRY-EXCEPT?)
#    (trans, rot) = self.tf_listener.lookupTransform('/map', '/camera_link', ts )
#    # Convert Transformation to R|T matrix form
#    RT = np.zeros( (3,4) )
#    RT[0:3,0:3] = tf.transformations.quaternion_matrix( rot )[0:3,0:3]
#    RT[:,-1] = trans
#    # Convert cloud to numpy array
    cloud_arr = pointclouds.pointcloud2_to_array( cloud_msg, split_rgb=True )
    # Get rid of z=nan points
    cloud_arr = cloud_arr[ np.isfinite( cloud_arr['z'] ) ]
    # Get the coords of points in the pc
#    xyz = np.ones( (cloud_arr.shape[0], 3) )
#    xyz[:,0] = cloud_arr['x']
#    xyz[:,1] = cloud_arr['y']
#    xyz[:,2] = cloud_arr['z']
#    # Transform the pointcloud
##    nxyz = RT.dot( xyz.T )
##    nxyz = nxyz.T
#    # Get the colors
#    clrs = 255 * np.ones( (xyz.shape[0],4) )
#    clrs[:,0] = cloud_arr['r']
#    clrs[:,1] = cloud_arr['g']
#    clrs[:,2] = cloud_arr['b']
#    clrs /= 255
    # Increment counter
#    xyz[:,[0,1]] = xyz[:,[1,0]]
#    xyz[:,2] *= -1
#    print 'imgPts', self.imgPts.shape, 'nxyz', nxyz.shape
#    print 'colors', self.colors.shape, 'clrs', clrs.shape
#    self.imgPts = np.concatenate( (self.imgPts, xyz) )
#    self.colors = np.concatenate( (self.colors, clrs) )
    
    # TEST:
#    now = rospy.Time.now()
#    print
#    print '%s\t%s\t%s' %( ts.to_sec(), now.to_sec(), now.to_sec()-ts.to_sec() )
##    print trans
#    print RT
#    print 'num points in cloud = (%s, %s)' %( xyz.shape[0], xyz.shape[1] )
##    print 'Transformation time = %s' %(toc - tic)
#    print
    print 'cloud %s rcvd' %(self.ctr)
    np.save('/home/grim4/Desktop/cloud_debug/cloud_%s.npy'%(self.ctr), cloud_arr)
    self.ctr += 1



    # TEST VISUALIZATION
#    print 'imgPts', self.imgPts.shape, 'xyz', xyz.shape
#    print 'colors', self.colors.shape, 'clrs', clrs.shape

#  def updatePlot( self ):
#    self.img.setData( pos=self.imgPts[::self.ctr],\
#                      color=self.colors[::self.ctr], size=DOTSIZE )
    
  def saveModel( self, blerg ):
#    print 'Extra arg = %s' %blerg
    print 'Saving model...'
    outcld = np.zeros( self.imgPts.shape[0], dtype=cldType )
    outcld['x'] = self.imgPts[:,0]
    outcld['y'] = self.imgPts[:,1]
    outcld['z'] = self.imgPts[:,2]
    outcld['r'] = self.colors[:,0]
    outcld['g'] = self.colors[:,1]
    outcld['b'] = self.colors[:,2]
    outcld['a'] = self.colors[:,3]
    np.save('mergedBatchClouds.npy', outcld )
    print 'Model saved.'
   
#def saveModel():
#  print 'Saving model...'
#  outcld = np.zeros( (listener.imgPts.shape[0],6) )
#  outcld[:,0:3] = listener.imgPts
#  outcld[:,3:] = listener.colors[:,0:3]
#  np.save('mergedBatchClouds.npy', outcld )
#  print 'Model saved.'

#global listener

if __name__ == '__main__':
  rospy.init_node('batch_cloud_listener', anonymous=True)
  listener = BatchCloudListener()
  rospy.spin()
#  while not rospy.is_shutdown():
#    time.sleep(2)
#    listener.updatePlot()
#  rospy.on_shutdown( listener.saveModel )
