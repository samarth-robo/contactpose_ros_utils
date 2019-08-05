#!/usr/bin/env python

import tf2_ros
import numpy as np
import rospy
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf import transformations as tx
import argparse
import sys
import os
import open3d
from copy import deepcopy
from render_depth_maps import render_depth_maps
from show_contactmap import apply_colormap

osp = os.path

def tform2matrix(tform):
  q = [tform.rotation.x, tform.rotation.y, tform.rotation.z, tform.rotation.w]
  T = tx.quaternion_matrix(q)
  T[:3, 3]  = [tform.translation.x, tform.translation.y, tform.translation.z]
  return T


def trimesh2open3d(mesh):
  omesh = open3d.TriangleMesh()
  omesh.vertices = open3d.Vector3dVector(np.asarray(mesh.vertices))
  omesh.triangles = open3d.Vector3iVector(np.asarray(mesh.faces))
  return omesh


def show_mesh(mesh):
  mesh = apply_colormap(mesh)
  mesh.compute_vertex_normals()
  mesh.compute_triangle_normals()
  open3d.draw_geometries([mesh])


class TextureMapper:
  def __init__(self, object_name, ignore_start=1e7, ignore_end=0):
    self.object_name = object_name
    self.camera_frame = 'boson_frame'
    self.object_frame = '{:s}_frame'.format(object_name)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    self.cv_bridge = CvBridge()
    self.im_sub = rospy.Subscriber("image_rect", Image, self.image_cb,
      queue_size=100)

    self.intrinsics = CameraInfo()
    self.images = []
    self.cTos = []

    self.last_msg_time = rospy.Time()
    self.start_time = rospy.Time()
    self.rx_initialized = False
    self.ignore_start = rospy.Duration(ignore_start)
    self.ignore_end = rospy.Duration(ignore_end)


  def image_cb(self, data):
    self.last_msg_time = data.header.stamp
    if not self.rx_initialized:
      self.start_time = self.last_msg_time
      self.rx_initialized = True
    else:
      if ((self.last_msg_time - self.start_time > self.ignore_end) and
          (self.last_msg_time - self.start_time < self.ignore_end)):
        rospy.loginfo_throttle(0.5, 'Ignoring')
        return

    rospy.loginfo_throttle(0.5, 'Received data')
    # get image
    try:
      cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logwarn(e)
      return

    # get object pose
    try:
      cTo = self.tf_buffer.lookup_transform(self.camera_frame, self.object_frame,
        data.header.stamp, rospy.Duration(1.0/20))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
    tf2_ros.ExtrapolationException) as e:
      rospy.logwarn(e)
      return
    self.cTos.append(tform2matrix(cTo.transform))
    self.images.append(cv_image)

    # get camera intrinsics for first callback
    if self.intrinsics.K[0] == 0:
      rospy.loginfo('Waiting for camera intrinsics...')
      self.intrinsics = rospy.wait_for_message('camera_info', CameraInfo,
        timeout=5.0)
      rospy.loginfo('Done')


  def texture_map(self, models_dir, output_filename, depth_thresh_for_visibility=1e-2,
      depth_thresh_for_discontinuity=0.035, max_vertex_normal_angle=70,
      mesh_scale=1e-3, show_textured_mesh=True, debug_mode=False, n_images=20):
    # read mesh file
    mesh_filename = osp.join(models_dir, '{:s}.ply'.format(self.object_name))
    mesh = open3d.read_triangle_mesh(mesh_filename)
    mesh.transform(np.eye(4) * mesh_scale)
    if not mesh.has_vertex_normals():
      mesh.compute_vertex_normals()

    intrinsic = open3d.PinholeCameraIntrinsic(self.intrinsics.width,
      self.intrinsics.height, self.intrinsics.K[0], self.intrinsics.K[4],
      self.intrinsics.K[2], self.intrinsics.K[5])

    # subsample the images
    assert len(self.images) == len(self.cTos)
    step = int(float(len(self.images)) / n_images)
    self.cTos = self.cTos[::step]
    self.images = self.images[::step]

    # get depth maps by rendering
    rospy.loginfo('Rendering depth maps...')
    depth_ims = render_depth_maps(mesh_filename, intrinsic, self.cTos,
      mesh_scale=mesh_scale)

    # create RGB-D images
    rgbds = []
    for depth_im, rgb_im, T in zip(depth_ims, self.images, self.cTos):
      depth_im = open3d.Image(depth_im)
      rgbds.append(open3d.create_rgbd_image_from_color_and_depth(
        open3d.Image(rgb_im), depth_im, convert_rgb_to_intensity=False))
      if debug_mode:
        pc = open3d.create_point_cloud_from_rgbd_image(rgbds[-1], intrinsic)
        tmesh = deepcopy(mesh)
        tmesh.transform(T)
        geoms = [pc]
        if show_textured_mesh:
          geoms.append(tmesh)
        open3d.draw_geometries(geoms)

    # create trajectory for texture mapping
    traj = open3d.PinholeCameraTrajectory()
    traj.extrinsic = open3d.Matrix4dVector(np.asarray(self.cTos))
    traj.intrinsic = intrinsic

    # do texture mapping!
    option = open3d.ColorMapOptmizationOption()
    option.maximum_iteration = 300
    option.depth_threshold_for_visiblity_check = depth_thresh_for_visibility
    option.depth_threshold_for_discontinuity_check = \
        depth_thresh_for_discontinuity
    option.half_dilation_kernel_size_for_discontinuity_map = 0
    option.max_angle_vertex_normal_camera_ray = max_vertex_normal_angle
    rospy.loginfo('Performing colormap optimization...')
    open3d.color_map_optimization(mesh, rgbds, traj, option)
    open3d.write_triangle_mesh(output_filename, mesh)
    print('{:s} written'.format(output_filename))
    if show_textured_mesh:
      show_mesh(mesh)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)
  parser.add_argument('--output_filename', default='texture_map.py')
  parser.add_argument('--models_dir',
    default='~/dropbox/contactdb_v2/data/ply_files_mm')
  parser.add_argument('--visibility_thresh', type=float, default=1e-2)
  parser.add_argument('--discontinuity_thresh', type=float, default=0.035)
  parser.add_argument('--vertex_normal_angle', type=float, default=70)

  rospy.init_node('texture_mapper')
  myargv = rospy.myargv(argv=sys.argv)

  args = parser.parse_args(myargv[1:])

  tm = TextureMapper(args.object_name)

  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    if tm.rx_initialized and \
      (rospy.Time.now() - tm.last_msg_time > rospy.Duration(2)):
      rospy.loginfo("Done, received {:d} frames".format(len(tm.images)))
      break
    rate.sleep()

  tm.texture_map(osp.expanduser(args.models_dir),
    osp.expanduser(args.output_filename),
    depth_thresh_for_visibility=args.visibility_thresh,
    depth_thresh_for_discontinuity=args.discontinuity_thresh,
    max_vertex_normal_angle=args.vertex_normal_angle)

  print('here')