"""
Converts Kinect2 calibration from JSON to a format understandable by iai_kinect2
"""
import numpy as np
import json
import os
import argparse
import cv2
from copy import deepcopy

osp = os.path


class Camera:
  def __init__(self):
    self.T = np.eye(4)
    self.K = np.eye(3)
    self.distortion = np.zeros(5)
    self.im_size = np.zeros(2)
    self.distance_offset = 0

  @property
  def projection(self):
    T = np.eye(4)
    T[:3, :3] = self.K
    return T

  @property
  def rotation(self):
    return np.eye(3)

  @property
  def dict(self):
    d = {
      'cameraMatrix': {
        'rows': 3,
        'cols': 3,
        'dt': 'd',
        'data': self.K.flatten().tolist()
      },
      'distortionCoefficients': {
        'rows': 1,
        'cols': 5,
        'dt': 'd',
        'data': self.distortion.tolist()
      },
      'rotation': {
        'rows': 3,
        'cols': 3,
        'dt': 'd',
        'data': self.rotation.tolist()
      },
      'projection': {
        'rows': 4,
        'cols': 4,
        'dt': 'd',
        'data': self.projection.tolist()
      }
    }
    return d

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


def convert(input_dir):
  input_dir = osp.join(input_dir)

  serials_filename = osp.join('..', 'calibrations', 'kinect_serial_numbers.txt')
  name2serial = {}
  with open(serials_filename, 'r') as f:
    for line in f:
      name, serial = line.strip().split()
      name2serial[name] = serial

  for kinect_name, serial in name2serial.items():
    # intrinsics
    output_dir = osp.join('..', 'calibrations', 'intrinsics', 'kinects', serial)
    if not osp.isdir(output_dir):
      os.makedirs(output_dir)

    json_filename = osp.join(input_dir, 'kinect_{:s}.json'.format(kinect_name))
    try:
      with open(json_filename, 'r') as f:
        data = json.load(f)
    except FileNotFoundError:
      print('skipping {:s}'.format(kinect_name))
      continue

    # read the JSON file
    camera_rgb = Camera()
    camera_ir = Camera()
    for camera_data in data:
      camera = Camera()
      camera.distance_offset = camera_data['DistanceOffset']

      camera_data = camera_data['Camera']
      camera.T = np.asarray(camera_data['ModelViewMatrix'])
      camera.K[0, 0] = camera_data['fx']
      camera.K[1, 1] = camera_data['fy']
      camera.K[0, 2] = camera_data['cx']
      camera.K[1, 2] = camera_data['cy']
      camera.distortion[0] = camera_data['k1']
      camera.distortion[1] = camera_data['k2']
      camera.distortion[2] = camera_data['p1']
      camera.distortion[3] = camera_data['p2']
      camera.distortion[4] = camera_data['p3']
      camera.im_size[0] = camera_data['ImageSizeX']
      camera.im_size[1] = camera_data['ImageSizeY']
      if 'color' in camera_data['SerialNo']:
        camera_rgb = deepcopy(camera)
        # iai_kinect2 expects matrix for HD image size
        camera_rgb.K[0, 0] *= 2.0
        camera_rgb.K[1, 1] *= 2.0
        camera_rgb.K[0, 2] *= 2.0
        camera_rgb.K[1, 2] *= 2.0
      else:
        camera_ir = deepcopy(camera)

    # output color YAML file
    filename = osp.join(output_dir, 'calib_color.yaml')
    camera_rgb.write(filename)

    # output IR YAML file
    filename = osp.join(output_dir, 'calib_ir.yaml')
    camera_ir.write(filename)

    # output depth calibration file
    filename = osp.join(output_dir, 'calib_depth.yaml')
    camera_ir.write_depth(filename)

    # output pose calibration file file
    T = np.dot(camera_rgb.T, np.linalg.inv(camera_ir.T))
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
    filename = osp.join(output_dir, 'calib_pose.yaml')
    f = cv2.FileStorage(filename, cv2.FileStorage_WRITE)
    f.write('rotation', R)
    f.write('translation', trans)
    f.write('essential', E)
    f.write('fundamental', F)
    f.release()
    print('{:s} written.'.format(filename))

    # extrinsics
    date = input_dir.split('/')[-1]
    json_filename = osp.join(input_dir,
                             'kinect_{:s}_optitrack.json'.format(kinect_name))
    try:
      with open(json_filename, 'r') as f:
        data = json.load(f)
    except FileNotFoundError:
      print('Kinect {:s} is not calibrated against Optitrack')
      continue
    for camera_data in data:
      camera_data = camera_data['Camera']
      if 'color' not in camera_data['SerialNo']:
        continue
      # modelview matrix (c_T_w)
      T = np.asarray(camera_data['ModelViewMatrix'])
      filename = osp.join('..', 'calibrations', 'extrinsics', date,
                          'kinect_{:s}.txt'.format(kinect_name))
      np.savetxt(filename, T)
      print('{:s} written.'.format(filename))

  # TODO: output YAML files


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--input_dir', required=True)

  args = parser.parse_args()
  convert(args.input_dir)