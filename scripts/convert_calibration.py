"""
Converts Kinect2 calibration from JSON to a format understandable by iai_kinect2
"""
import numpy as np
import json
import os
import argparse
import cv2
from copy import deepcopy
import camera_info_manager as cinfo_manager
from sensor_msgs.msg import CameraInfo

osp = os.path


class Camera:
  def __init__(self):
    self.cTw = np.eye(4)
    self.K = np.eye(3)
    self.distortion = np.zeros(5)
    self.im_size = np.zeros(2)
    self.distance_offset = 0
    self.serial = ''

  @property
  def projection(self):
    P = np.eye(4)
    P[:3, :3] = self.K
    return P

  @property
  def rotation(self):
    return np.eye(3)

  def write(self, output_filename):
    f = cv2.FileStorage(output_filename, cv2.FileStorage_WRITE)
    f.write('cameraMatrix', self.K)
    f.write('distortionCoefficients', self.distortion)
    f.write('rotation', self.rotation)
    f.write('projection', self.projection)
    f.release()
    print('{:s} written.'.format(output_filename))

  def write_depth(self, output_filename):
    f = cv2.FileStorage(output_filename, cv2.FileStorage_WRITE)
    f.write('depthShift', self.distance_offset)
    f.release()
    print('{:s} written.'.format(output_filename))

  def write_cinfo(self, output_filename, frame_id, camera_name):
    cinfo = CameraInfo()
    cinfo.header.frame_id = frame_id
    cinfo.width = int(self.im_size[0])
    cinfo.height = int(self.im_size[1])
    for i in range(9):
      cinfo.K[i] = float(self.K.flat[i])
      cinfo.R[i] = float(self.rotation.flat[i])
    cinfo.distortion_model = 'plumb_bob'
    cinfo.D.extend(self.distortion.tolist())
    for i in range(12):
      cinfo.P[i] = float(self.projection.flat[i])

    return cinfo_manager.saveCalibration(cinfo, output_filename, camera_name)


def json2camera(camera_data):
  """
  :param camera_data: dict from parsing json
  :return: Camera object
  """
  camera = Camera()
  camera.distance_offset = camera_data['DistanceOffset']

  camera_data = camera_data['Camera']
  camera.cTw = np.asarray(camera_data['ModelViewMatrix'])
  camera.K[0, 0] = camera_data['fx']
  camera.K[1, 1] = camera_data['fy']
  camera.K[0, 2] = camera_data['cx']
  camera.K[1, 2] = camera_data['cy']
  camera.distortion[0] = camera_data['k1']
  camera.distortion[1] = camera_data['k2']
  camera.distortion[2] = camera_data['p1']
  camera.distortion[3] = camera_data['p2']
  camera.distortion[4] = camera_data['k3']
  camera.im_size[0] = camera_data['ImageSizeX']
  camera.im_size[1] = camera_data['ImageSizeY']
  camera.serial = camera_data['SerialNo']

  return camera


def convert(input_dir, date):
  input_dir = osp.join(input_dir)

  serials_filename = osp.join('..', 'calibrations', 'kinect_serial_numbers.txt')
  name2serial = {}
  with open(serials_filename, 'r') as f:
    for line in f:
      name, serial = line.strip().split()
      name2serial[name] = serial

  # extrinsics
  extrinsics_output_dir = osp.join('..', 'calibrations', 'extrinsics', date)
  if not osp.isdir(extrinsics_output_dir):
    os.makedirs(extrinsics_output_dir)

  for kinect_name, serial in name2serial.items():
    # intrinsics
    intrinsics_output_dir = osp.join('..', 'calibrations', 'intrinsics', 'kinects',
      serial)
    if not osp.isdir(intrinsics_output_dir):
      os.makedirs(intrinsics_output_dir)

    json_filename = osp.join(input_dir, 'intrinsics',
                             'kinect_{:s}.json'.format(kinect_name))
    try:
      with open(json_filename, 'r') as f:
        data = json.load(f)
    except IOError:
      print('skipping {:s}'.format(kinect_name))
      continue

    # read the JSON file
    camera_rgb = Camera()
    camera_ir = Camera()
    for camera_data in data:
      camera = json2camera(camera_data)
      if 'color' in camera.serial:
        camera_rgb = deepcopy(camera)
        # iai_kinect2 expects matrix for HD image size
        camera_rgb.K[0, 0] *= 2.0
        camera_rgb.K[1, 1] *= 2.0
        camera_rgb.K[0, 2] *= 2.0
        camera_rgb.K[1, 2] *= 2.0
      else:
        camera_ir = deepcopy(camera)

    # output color YAML file
    filename = osp.join(intrinsics_output_dir, 'calib_color.yaml')
    camera_rgb.write(filename)

    # output IR YAML file
    filename = osp.join(intrinsics_output_dir, 'calib_ir.yaml')
    camera_ir.write(filename)

    # output pose calibration file file
    T = np.dot(camera_rgb.cTw, np.linalg.inv(camera_ir.cTw))
    R = T[:3, :3]
    trans = T[:3, 3] / 1000.0
    trans_skew = np.asarray([
      [0, -trans[2], trans[1]],
      [trans[2], 0, -trans[0]],
      [-trans[1], trans[0], 0]
    ])
    E = np.dot(trans_skew, R)
    F = np.dot(np.dot(np.linalg.inv(camera_rgb.K), E), camera_ir.K)
    F /= F[2 , 2]
    filename = osp.join(intrinsics_output_dir, 'calib_pose.yaml')
    f = cv2.FileStorage(filename, cv2.FileStorage_WRITE)
    f.write('rotation', R)
    f.write('translation', trans)
    f.write('essential', E)
    f.write('fundamental', F)
    f.release()
    print('{:s} written.'.format(filename))

    json_filename = osp.join(input_dir, 'extrinsics', date,
                             'kinect_{:s}_optitrack.json'.format(kinect_name))
    try:
      with open(json_filename, 'r') as f:
        data = json.load(f)
    except IOError:
      print('Kinect {:s} is not calibrated against Optitrack, using image-based'
            'depth offset')
      # Use the depth offset from image-based calibration
      filename = osp.join(intrinsics_output_dir, 'calib_depth.yaml')
      camera_ir.write_depth(filename)
      continue

    for camera_data in data:
      camera = json2camera(camera_data)
      if 'color' in camera.serial:  # output the pose of Opitrack w.r.t. Kinect
        filename = osp.join(extrinsics_output_dir,
                            'kinect_{:s}.txt'.format(kinect_name))
        np.savetxt(filename, camera.cTw)
        print('{:s} written.'.format(filename))
      elif 'depth' in camera.serial:  # output depth calibration file
        filename = osp.join(intrinsics_output_dir, 'calib_depth.yaml')
        camera.write_depth(filename)

  # Thermal camera files
  # for thermal camera, the extrinsics calibration also refines intrinsics,
  # so we take intrinsics from the extrinsics file
  json_filename = osp.join(input_dir, 'extrinsics', date, 'boson_optitrack.json')
  try:
    with open(json_filename, 'r') as f:
      data = json.load(f)
  except IOError:
    print('Could not find Boson extrinsics file {:s}'.format(json_filename))
  camera = json2camera(data[0])
  # write intrinsics
  filename = 'package://contactdb_utils/calibrations/intrinsics/boson.yaml'
  if camera.write_cinfo(filename, 'boson_frame', 'boson'):
    print("{:s} written".format(filename))
  else:
    print('Could not write {:s}'.format(filename))
  # write extrinsics
  filename = osp.join(extrinsics_output_dir, 'boson.txt')
  np.savetxt(filename, camera.cTw)
  print('{:s} written.'.format(filename))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--input_dir', required=True)
  parser.add_argument('--date', required=True)

  args = parser.parse_args()
  convert(args.input_dir, args.date)