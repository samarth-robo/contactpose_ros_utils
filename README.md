# ContactPose ROS

ROS code used for recording the ContactPose dataset released in the following paper:

[ContactPose: A Dataset of Grasps with Object Contact and Hand Pose](https://contactpose.cc.gatech.edu) - 

[Samarth Brahmbhatt](https://samarth-robo.github.io/),
[Chengcheng Tang](https://scholar.google.com/citations?hl=en&user=WbG27wQAAAAJ),
[Christopher D. Twigg](https://scholar.google.com/citations?hl=en&user=aN-lQ0sAAAAJ),
[Charles C. Kemp](http://charliekemp.com/), and
[James Hays](https://www.cc.gatech.edu/~hays/),

**ECCV 2020**.

- [Explore the dataset](https://contactpose.cc.gatech.edu/contactpose_explorer.html)
- [hand-object contact ML code](https://github.com/samarth-robo/ContactPose-ML)

## Citation
```
@InProceedings{Brahmbhatt_2020_ECCV,
author = {Brahmbhatt, Samarth and Tang, Chengcheng and Twigg, Christopher D. and Kemp, Charles C. and Hays, James},
title = {{ContactPose}: A Dataset of Grasps with Object Contact and Hand Pose},
booktitle = {The European Conference on Computer Vision (ECCV)},
month = {August},
year = {2020}
}
```

## Notes
Tested on Ubuntu 16.04 + ROS Kinetic. Features:
- Records 3 Kinects + TF + Thermal camera simultaneously to bag files
- Controls the Cowtech Ciclops turntable
- Uses [low-latency FLRI Boson thermal camera drivers](https://github.com/samarth-robo/cv_camera)

Raise a GitHub issue for questions, sorry the code is not well documented, but I am happy to assist with questions.

### Dependencies
See [`docs/deps.md`](docs/deps.md)

### Entry Points
- [`launch/record_object.launch`](launch/record_object.launch): High level launch file for recording a grasp. Starts 3 Kinects and the thermal camera, controls the turnrable, records data to bag file.
- [`scripts/embed_marker_points.py`](scripts/embed_marker_points.py) and [`scripts/embed_marker_points_docker.sh`](scripts/embed_marker_points_docker.sh): Edit object 3D models to create small cylindrical cavities for marker embedding. Save the locations of those cavities.
- [`scripts/align_optitrack_rigid_body.py`](scripts/align_optitrack_rigid_body.py): Given the marker constellation from Optitrack Motive tracking software, and the marker locations in the object coordinate system (generated by [`scripts/embed_marker_points.py`](scripts/embed_marker_points.py)), find the 6-DOF transformation that aligns the two. This is necessary because the coordinate system assigned by Motive to the marker constellation is not the same as the object coordinate system that is used in all of ContactPose code.
- [`scripts/setuo_kinect.sh`](scripts/setuo_kinect.sh): Increase the USB filsystem memory, required for operating multiple kinects on the same computer. See [this guide](https://github.com/OpenKinect/libfreenect2/wiki/Troubleshooting#multiple-kinects-try-increasing-usbfs-buffer-size).
