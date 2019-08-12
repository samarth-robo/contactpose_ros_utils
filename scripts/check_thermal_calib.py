#!/usr/bin/env python

import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from tf import transformations as tx
import cv2
import numpy as np
import argparse
import os

osp = os.path


def tx_quat(rotation):
  q = np.asarray([
    rotation.x,
    rotation.y,
    rotation.z,
    rotation.w
  ])
  return q


class JointProjector:
  def __init__(self, object_name, markers_dir):
    self.image_sub = rospy.Subscriber("image_in", Image, self.callback,
      queue_size=1, buff_size=2**24)
    self.image_pub = rospy.Publisher("image_out", Image, queue_size=1)
    self.bridge = CvBridge()

    # TF information
    self.camera_frame_id = "boson_frame"
    self.object_frame_id = "{:s}_frame".format(object_name)
    self.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5))
    self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

    # camera intrinsics
    try:
      cinfo = rospy.wait_for_message("camera_info", CameraInfo, 2)
    except rospy.ROSException as e:
      rospy.logerr("Could not listen to camera_info topic")
      sys.exit(-1)
    self.K = np.asarray(cinfo.K).reshape((3, 3))

    # marker locations
    markers_filename = osp.join(markers_dir,
                                '{:s}_marker_locations.txt'.format(object_name))
    self.oPs = np.loadtxt(markers_filename, delimiter=',')[:, :3] / 1000


  def callback(self, img_msg):
    # read image
    try:
      im_in = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
      im_in = np.asarray(im_in)
    except CvBridgeError as e:
      print(e)

    # read transform
    try:
      wTo1 = self.tfBuffer.lookup_transform('optitrack_frame', self.object_frame_id, img_msg.header.stamp-rospy.Duration(0.1), rospy.Duration(2))
      wTo2 = self.tfBuffer.lookup_transform('optitrack_frame', self.object_frame_id, img_msg.header.stamp, rospy.Duration(2))
      e1 = np.rad2deg(tx.euler_from_quaternion(tx_quat(wTo1.transform.rotation)))
      e2 = np.rad2deg(tx.euler_from_quaternion(tx_quat(wTo2.transform.rotation)))
      ve = (180 - np.abs(np.abs(e2-e1) - 180)) / 0.1
      # nominal velocity about the up vector (Y axis) is 50 deg / s
      delay = rospy.Duration(ve[1]/50.0 * 0.2)
      cTo = self.tfBuffer.lookup_transform(self.camera_frame_id,
                                           self.object_frame_id,
                                           img_msg.header.stamp-delay,
        rospy.Duration(2))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
    tf2_ros.ExtrapolationException) as e:
      rospy.logerr(e)
      return

    q = tx_quat(cTo.transform.rotation)
    T = tx.quaternion_matrix(q)
    T[0, 3] = cTo.transform.translation.x
    T[1, 3] = cTo.transform.translation.y
    T[2, 3] = cTo.transform.translation.z
    oPs = np.vstack((self.oPs.T, np.ones(len(self.oPs))))
    cPs = np.dot(T, oPs)
    cPs = np.dot(self.K, cPs[:3])
    cPs[:2] /= cPs[2]
    cPs = cPs[:2].T.astype(int)

    # create the output image
    im_out = im_in.copy()
    color = (0, 255, 0)
    for idx, cP in enumerate(cPs):
      im_out = cv2.circle(im_out, tuple(cP), 3, color, -1)

    # publish
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_out, "bgr8"))
    except CvBridgeError as e:
      print(e)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name')
  parser.add_argument('--markers_dir',
                      default=osp.join('~', 'dropbox', 'contactdb_v2', 'data',
                                       'stl_files_notched'))

  rospy.init_node('joint_projector')
  myargv = rospy.myargv(argv=sys.argv)
  args = parser.parse_args(myargv[1:])

  jp = JointProjector(args.object_name, osp.expanduser(args.markers_dir))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")