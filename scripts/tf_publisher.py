#!/usr/bin/env python
import sys
import roslib
package_name = 'contactdb_utils'
roslib.load_manifest(package_name)
import rospy
import tf2_ros
from tf_conversions import transformations as tx
import numpy as np
import pickle
import os.path as osp
import rospkg
from geometry_msgs.msg import TransformStamped
import argparse


def get_tform(R, T, parent_frame, child_frame):
  cam_rot = np.eye(4)
  cam_rot[:3, :3] = R
  cam_q = tx.quaternion_from_matrix(cam_rot)

  # construct the static transform
  tform = TransformStamped()
  tform.header.stamp = rospy.Time.now()
  tform.header.frame_id = parent_frame
  tform.child_frame_id = child_frame
  tform.transform.translation.x = float(T[0])
  tform.transform.translation.y = float(T[1])
  tform.transform.translation.z = float(T[2])
  tform.transform.rotation.x = cam_q[0]
  tform.transform.rotation.y = cam_q[1]
  tform.transform.rotation.z = cam_q[2]
  tform.transform.rotation.w = cam_q[3]
  return tform


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--date', required=True)

  rospy.init_node('contactdb_tf_broadcaster')
  rospack = rospkg.RosPack()

  myargv = rospy.myargv(argv=sys.argv)
  args = parser.parse_args(myargv[1:])

  calib_dir = osp.join(rospack.get_path(package_name), 'calibrations')
  name2serial = {}
  with open(osp.join(calib_dir, 'kinect_serial_numbers.txt'), 'r') as f:
    for line in f:
      name, serial = line.strip().split(" ")
      name2serial[name] = serial

  calib_dir = osp.join(calib_dir, 'extrinsics', args.date)
  tforms = []
  for kinect_name, serial in name2serial.items():
    k_T_w = np.loadtxt(osp.join(calib_dir,
                                'kinect_{:s}.txt'.format(kinect_name)))
    k_T_w[:3, 3] /= 1000.0
    w_T_k = np.linalg.inv(k_T_w)

    tform = get_tform(w_T_k[:3, :3], w_T_k[:3, 3], 'optitrack_frame',
                      'kinect2_{:s}_link'.format(kinect_name))
    tforms.append(tform)

  # thermal camera
  k_T_w = np.loadtxt(osp.join(calib_dir, 'boson.txt'))
  k_T_w[:3, 3] /= 1000.0
  w_T_k = np.linalg.inv(k_T_w)
  tform = get_tform(w_T_k[:3, :3], w_T_k[:3, 3], 'optitrack_frame', 'boson_frame')
  tforms.append(tform)

  # rviz transformation
  rviz_tform = get_tform(tx.euler_matrix(np.pi/2.0, 0, 0)[:3, :3], np.zeros(3),
                         'rviz_frame', 'optitrack_frame')
  tforms.append(rviz_tform)

  # send
  broadcaster = tf2_ros.StaticTransformBroadcaster()
  broadcaster.sendTransform(tforms)
  rospy.spin()
