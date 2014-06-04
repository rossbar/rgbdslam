import numpy as np
cld = np.load('mergedBatchClouds.npy')
pts = cld.flatten()
import time
KHFOV = 57      # Kinect horizontal FOV (degrees)
KVFOV = 43      # Kinect vertical FOV (degrees)
NHPIX = 640     # Number of horizontal pixels (columns)
NVPIX = 480     # Number of vertical pixels (rows)
MAX_DEPTH = 6   # Maximum depth that can be sensed by the kinect (m)
IMG_SCALER = 10 # Fudge-factor for making the X and Y scales similar to the
                # depth scale
ALPHA = 0.3     # Alpha-value for radiation image overlay
DOTSIZE = 0.1   # Size of dots for glScatter images
DS = 5		# Downsampling factor
import PySide
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.show()
w.setWindowTitle('Kinect Points')
pos = np.zeros( (pts.shape[0],3) )
pos[:,0] = pts['x']
pos[:,1] = pts['y']
pos[:,2] = pts['z']
clrs = 255*np.ones( (pos.shape[0],4) )
clrs[:,0] = pts['r']
clrs[:,1] = pts['g']
clrs[:,2] = pts['b']
clrs[:,3] = pts['a']
#clrs /= 255
img = gl.GLScatterPlotItem( pos=pos[::DS], color=clrs[::DS], size=DOTSIZE )
w.addItem(img)

