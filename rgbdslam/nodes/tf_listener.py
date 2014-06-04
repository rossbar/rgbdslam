#!/usr/bin/env python  
import roslib
roslib.load_manifest('rgbdslam')
import rospy
import math
import tf
#import turtlesim.msg
#import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    print '#timestamp	tx	ty	tz	R00	R01	R02	R10	R11	R12	R20	R21	R22'
    while not rospy.is_shutdown():
        try:
            ts = rospy.get_time()
            (trans,rot) = listener.lookupTransform('/camera_link', '/map',\
                                                   rospy.Time(0) )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

#        # Print what you find
#        print
#        print 'Timstamp: ', ts
#        print 'Translation = ', trans
        rot = tf.transformations.quaternion_matrix( rot )
#        print 'Rotation = \n', rot
#        print
        print '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t' %(ts,trans[0],trans[1],trans[2],rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2])

        rate.sleep()
