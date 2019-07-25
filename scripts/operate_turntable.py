#!/usr/bin/python
"""
ROS node that operates the turntable in the CowTech Ciclop 3D scanner.
It sets the current position as 0 degrees, rotates the turntable through
360 degrees and publishes its pose continuously as it is rotating
"""
import logging
from horus.engine.driver import board
import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import argparse
from tf import transformations as tx
import tf2_ros
import numpy as np
import sys
import subprocess
import os
from PyQt5.QtWidgets import QApplication
import cv2
import multiprocessing as mp
from recording_gui import RecordingGUI

osp = os.path


class StatusWindowProcess(mp.Process):
  def __init__(self, status_var):
    mp.Process.__init__(self)
    self.status_var = status_var
    self.win_name = 'status'

  def _get_im(self, status):
    """
    :param status: 0: red 1: green
    :return:
    """
    im = np.zeros((60, 60, 3), dtype=np.uint8)
    if status == 1:
      im[:, :, 1] = 255
    elif status == 0:
      im[:, :, 2] = 255

    return im

  def run(self):
    while True:
      with self.status_var.get_lock():
        if self.status_var.value == 2:
          break
        im = self._get_im(self.status_var.value)
      cv2.imshow(self.win_name, im)
      cv2.waitKey(30)


class RosbagRecord:
  """
  from https://gist.github.com/marco-tranzatto/8be49b81b1ab371dcb5d4e350365398a
  """
  def __init__(self, record_folder, kinect_res, hand_pose=False):
    self.record_script = 'hand_pose_record.sh' if hand_pose else \
        'contactdb_record.sh'
    self.bag_filename_prefix = '-hand-pose' if hand_pose else ''
    self.node_name = 'hand_pose_recorder' if hand_pose else \
        'contactdb_recorder'
    if not osp.isfile(self.record_script):
      rospy.logerr('Script file {:s} does not exist'.
                   format(osp.join(os.getcwd(), self.record_script)))
      return
    self.record_folder = osp.expanduser(record_folder)
    if not osp.exists(self.record_folder):
      os.makedirs(self.record_folder)
      rospy.loginfo('Directory {:s} created'.format(self.record_folder))
    self.kinect_res = kinect_res

  def start_recording(self, object_name):
    # Start recording.
    command = "source {:s} {:s}/{:s}{:s} {:s} {:s}".format(self.record_script,
      self.record_folder, object_name, self.bag_filename_prefix, self.kinect_res,
      self.node_name)
    self.p = subprocess.Popen(command, stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              shell=True, cwd=os.getcwd(),
                              executable='/bin/bash')
    rospy.loginfo(rospy.get_name() + ' start recording.')

  def _terminate_ros_node(self):
    # Adapted from
    # http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    # https://answers.ros.org/question/275441/start-and-kill-rosbag-record-from-bash-shell-script/
    list_cmd = subprocess.Popen("rosnode list", shell=True,
                                stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for node in list_output.split("\n"):
      if node.startswith('/{:s}'.format(self.node_name)):
        os.system("rosnode kill " + node)

  def stop_recording_handler(self):
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    self._terminate_ros_node()
    if hasattr(self, 'p'):
      o = str(self.p.stdout.read())
      e = str(self.p.stderr.read())
      rospy.loginfo('recorder script stdout: {:s}\nstderr: {:s}'.
          format(o, e))


class TurnTableOperator(object):
  def __init__(self, serial_port, contactdb_recorder, hand_pose_recorder,
      step=3, motor_speed=150, motor_acceleration=200):
    """"
    :param serial_port:
    :param {contactdb,hand_pose}_recorder: Object of the RosbagRecord class
    :param step:
    :param motor_speed:
    :param motor_acceleration:
    """
    self.step = step
    self.contactdb_recorder = contactdb_recorder
    self.hand_pose_recorder = hand_pose_recorder
    self.tt_base_pose = TransformStamped()  # TODO: fill by reading
    self.tt_base_pose.header.frame_id = 'optitrack_frame'
    self.tt_base_pose.child_frame_id = 'turntable_base'
    self.tt_base_pose.transform.rotation.w = 1
    self.tt_bcaster = tf2_ros.TransformBroadcaster()
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.boson_ffc_pub = rospy.Publisher('/flir/command/thermal/runffc', String,
                                         queue_size=1)

    # status window
    self.status_var = mp.Value('b', 0)
    self.status_process = StatusWindowProcess(self.status_var)
    self.status_process.daemon = True
    self.status_process.start()

    # logging for the Arduino driver
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    board.logger.addHandler(ch)

    # init and connect the driver
    self.arduino = board.Board()
    self.arduino.serial_name = serial_port
    try:
      self.arduino.connect()
    except Exception as e:
      rospy.logerr('Could not connect to turntable: {:s}'.format(e))
      raise e

    # init and configure the motor
    self.arduino.motor_invert(True)
    self.arduino.motor_enable()
    self.arduino.motor_reset_origin()
    self.arduino.motor_speed(motor_speed)
    self.arduino.motor_acceleration(motor_acceleration)
    rospy.sleep(0.5)

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.disconnect()

  def publish_tfs(self, tt_angle):
    t = TransformStamped()  # pose of turntable_frame w.r.t. turntable_base
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'turntable_base'
    t.child_frame_id  = 'turntable_frame'
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    q = tx.quaternion_from_euler(0, 0, np.deg2rad(tt_angle))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    self.tt_bcaster.sendTransform([t, self.tt_base_pose])

  def _set_recording_status(self, status):
    with self.status_var.get_lock():
      if status:
        self.status_var.value = 1
      else:
        self.status_var.value = 0

  def record_hand_pose(self, object_name):
    self._set_recording_status(True)
    self.hand_pose_recorder.start_recording(object_name)

  def stop_record_hand_pose(self):
    self.hand_pose_recorder.stop_recording_handler()
    self._set_recording_status(False)

  def run_turntable(self, object_name):
    # first stop the hand pose recorder node if it is active
    self.stop_record_hand_pose()
    # perform Boson FFC
    self.boson_ffc_pub.publish("00000001")
    # start recording for contactdb
    self._set_recording_status(True)
    angle = 0
    self.contactdb_recorder.start_recording(object_name)
    rospy.sleep(1.5)
    while angle < 360:
      rospy.loginfo('Angle = {:d}'.format(angle))
      self.publish_tfs(tt_angle=angle)
      rospy.sleep(0.5)
      self.arduino.motor_move(self.step)
      angle += self.step
      rospy.sleep(0.5)
    self.contactdb_recorder.stop_recording_handler()
    self._set_recording_status(False)

  def disconnect(self):
    self.arduino.disconnect()
    rospy.loginfo('Disconnected from Arduino')
    self.contactdb_recorder.stop_recording_handler()
    self.hand_pose_recorder.stop_recording_handler()
    rospy.loginfo('Waiting to join the status-window process...')
    with self.status_var.get_lock():
      self.status_var.value = 2
    self.status_process.join()
    rospy.loginfo('Done')

  def can_transform(self, object_name):
    try:
      self.tf_buffer.lookup_transform('optitrack_frame',
                                      '{:s}_frame_optitrack'.format(object_name),
                                      rospy.Time.now(), rospy.Duration(0.5))
      return True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
    tf2_ros.ExtrapolationException) as e:
      rospy.logwarn('Could not lookup the transform for {:s}: {:s}. Is Optitrack '
                    'running and streaming?'.format(object_name, e))
      return False


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--serial_port', type=str, default='/dev/ttyUSB0',
                      help='Serial port of the Arduino')
  parser.add_argument('--data_dir', type=str,
                      default=osp.join('..', 'data', 'contactdb_data'))
  parser.add_argument('--kinect_res', type=str, default='qhd')

  rospy.init_node('turntable_operator')
  myargv = rospy.myargv(argv=sys.argv)
  args = parser.parse_args(myargv[1:])

  contactdb_recorder = RosbagRecord(args.data_dir, args.kinect_res,
      hand_pose=False)
  hand_pose_recorder = RosbagRecord(args.data_dir, args.kinect_res,
      hand_pose=True)
  with TurnTableOperator(contactdb_recorder=contactdb_recorder,
      hand_pose_recorder=hand_pose_recorder, serial_port=args.serial_port,
      step=40) as tt:
    def record_cb(object_name, hand_pose=False):
      if object_name != 'hands':
        while not tt.can_transform(object_name):
          pass
      if hand_pose:
        tt.record_hand_pose(object_name)
      else:
        if tt.arduino._is_connected:
          tt.run_turntable(object_name)
        else:
          rospy.logwarn('Turntable is not connected, not recording')
    def contactdb_cb(object_name):
      record_cb(object_name=object_name, hand_pose=False)
    def hand_pose_cb(object_name):
      record_cb(object_name=object_name, hand_pose=True)

    # Qt stuff
    app = QApplication(sys.argv)
    gui = RecordingGUI(contactdb_recording_cb=contactdb_cb,
      hand_pose_recording_cb=hand_pose_cb,
      stop_hand_pose_cb=tt.stop_record_hand_pose)
    retval = app.exec_()
  sys.exit(retval)
