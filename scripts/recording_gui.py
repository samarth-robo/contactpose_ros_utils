import sys
from PyQt5.QtCore import pyqtSlot, Qt, QThread, QObject, pyqtSignal
from PyQt5.QtWidgets import *
from functools import partial
import time
import tf2_ros
import rospy
import yaml


class FramesThread(QThread, QObject):
  object_signal = pyqtSignal(str)

  def __init__(self):
    QThread.__init__(self)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

  def __del__(self):
    self.wait()

  def run(self):
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():  # get current object name from TF
      frames = yaml.load(self.tf_buffer.all_frames_as_yaml())
      object_names = []
      for frame, frame_info in frames.items():
        if 'optitrack' not in frame:
          continue
        if frame == 'optitrack_frame':
          continue
        if rospy.Time.now() - rospy.Time(frame_info['most_recent_transform']) <\
            rospy.Duration(0.1):
          object_names.append(frame.replace('_frame_optitrack', ''))
      if len(object_names) == 0:
        self.object_signal.emit('hands')
      elif len(object_names) > 1:
        rospy.logwarn('Optitrack is tracking multiple objects: {:s}'.
          format(' '.join(object_names)))
      else:
        self.object_signal.emit(object_names[0])
      rate.sleep()


class RecordingGUI(QWidget):
  def __init__(self, contactdb_recording_cb, hand_pose_recording_cb,
      stop_hand_pose_cb, *args, **kwargs):
    super(RecordingGUI, self).__init__(*args, **kwargs)
    self.contactdb_recording_cb = contactdb_recording_cb
    self.hand_pose_recording_cb = hand_pose_recording_cb
    self.stop_hand_pose_cb = stop_hand_pose_cb

    self.setWindowTitle('Select objects')
    self.current_object = 'hands'
    self.frames_thread = FramesThread()
    self.frames_thread.object_signal.connect(self.set_current_object)
    self.frames_thread.start()

    self.object_grid = QGridLayout()
    self.object_textbox = QLineEdit(parent=self)
    self.object_textbox.setReadOnly(True)
    self.object_textbox.setAlignment(Qt.AlignCenter)
    self._update_object_textbox()
    self.object_grid.addWidget(self.object_textbox, 0, 0)

    # record buttons
    self.button_grid = QGridLayout()
    self.hand_pose_record_button = QPushButton('Record Hand Pose',
        parent=self)
    self.hand_pose_record_button.clicked.connect(partial(self.record_cb,
      hand_pose=True))
    self.button_grid.addWidget(self.hand_pose_record_button, 0, 0)
    self.contactdb_record_button = QPushButton('Record ContactDB',
        parent=self)
    self.contactdb_record_button.clicked.connect(partial(self.record_cb,
      hand_pose=False))
    self.button_grid.addWidget(self.contactdb_record_button, 0, 1)
    self.stop_hand_pose_record_button = QPushButton('Stop Hand Pose',
      parent=self)
    self.stop_hand_pose_record_button.clicked.connect(self.stop_hand_pose_slot)
    self.button_grid.addWidget(self.stop_hand_pose_record_button, 0, 2)

    # main layout
    self.vlayout = QVBoxLayout()
    self.vlayout.addLayout(self.object_grid)
    self.vlayout.addLayout(self.button_grid)
    self.setLayout(self.vlayout)

    self.show()

  def _update_object_textbox(self):
    self.object_textbox.setText(self.current_object)

  def set_current_object(self, current_object):
    self.current_object = current_object
    self._update_object_textbox()

  @pyqtSlot()
  def on_click(self, name):
    self.current_object = name
    print('Current object = {:s}'.format(name))

  @pyqtSlot()
  def record_cb(self, hand_pose):
    if hand_pose:
      print('Recording {:s} hand pose...'.format(self.current_object))
      self.hand_pose_recording_cb(object_name=self.current_object)
    else:
      print('Recording {:s} ContactDB...'.format(self.current_object))
      self.contactdb_recording_cb(object_name=self.current_object)
    print('Done')

  @pyqtSlot()
  def stop_hand_pose_slot(self):
    print('Stopping hand pose recording...')
    self.stop_hand_pose_cb()
    print('Done')


if __name__ == '__main__':
  rospy.init_node("RecordingGUI")
  # create our window
  app = QApplication(sys.argv)

  # Show the window and run the app
  def recording_fn(object_name):
    time.sleep(1)
  def stop_recording_fn():
    time.sleep(1)
  gui = RecordingGUI(recording_fn, recording_fn, stop_recording_fn)
  retval = app.exec_()
  sys.exit(retval)
