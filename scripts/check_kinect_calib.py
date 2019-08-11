#!/usr/bin/env python
import sys
import os
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import argparse

osp = os.path


def make_point(p):
  out = Point()
  out.x = p[0]
  out.y = p[1]
  out.z = p[2]
  return out


def publish(object_name, markers_dir, point_size=7e-3):
  # marker locations
  markers_filename = osp.join(markers_dir,
                              '{:s}_marker_locations.txt'.format(object_name))
  oMs = np.loadtxt(markers_filename, delimiter=',')[:, :3] / 1000

  # TF stuff
  ns = 'object_markers'.format(object_name)
  frame_id = '{:s}_frame'.format(object_name)

  joint_color = ColorRGBA()
  joint_color.r = 1
  joint_color.a = 1
  marker_color = ColorRGBA()
  marker_color.g = 1
  marker_color.a = 1

  # common between all markers
  pts = Marker()
  pts.ns = ns
  pts.header.frame_id = frame_id
  pts.action = Marker.ADD
  pts.pose.orientation.w = 1
  # create joint points and marker points
  pts.id = 0
  pts.type = Marker.POINTS
  pts.scale.x = pts.scale.y = point_size
  for oM in oMs:  # markers
    pts.points.append(make_point(oM))
    pts.colors.append(marker_color)

  publisher = rospy.Publisher("visualization_marker", Marker, queue_size=1)
  rate = rospy.Rate(120)
  rospy.loginfo('Publishing hand and object markers')
  while not rospy.is_shutdown():
    publisher.publish(pts)
    rate.sleep()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name')
  parser.add_argument('--markers_dir',
                      default=osp.join('~', 'dropbox', 'contactdb_v2', 'data',
                                       'stl_files_notched'))

  rospy.init_node('object_marker_pub')
  myargv = rospy.myargv(argv=sys.argv)
  args = parser.parse_args(myargv[1:])

  publish(args.object_name, osp.expanduser(args.markers_dir))