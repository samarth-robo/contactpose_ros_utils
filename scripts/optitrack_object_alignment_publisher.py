#!/usr/bin/env python
import sys
import roslib
package_name = 'contactpose_ros_utils'
roslib.load_manifest(package_name)
import rospy
import tf2_ros
import numpy as np
import os.path as osp
import rospkg
import argparse
from tf_publisher import get_tform


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)

  rospy.init_node('optitrack_object_alignment_publisher')
  rospack = rospkg.RosPack()

  myargv = rospy.myargv(argv=sys.argv)
  args = parser.parse_args(myargv[1:])

  filename = osp.join(rospack.get_path(package_name), 'data', 'contactdb_data',
                      'optitrack_rigid_bodies',
                      'optitrack_{:s}_alignment.txt'.format(args.object_name))
  T = np.loadtxt(filename)
  tform = get_tform(T[:3, :3], T[:3, 3],
                    '{:s}_frame_optitrack'.format(args.object_name),
                    '{:s}_frame'.format(args.object_name))

  # send
  broadcaster = tf2_ros.StaticTransformBroadcaster()
  broadcaster.sendTransform(tform)
  rospy.spin()
