# [ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu)
# Dependencies
1. Download [this fork](https://github.com/samarth-robo/Open3D/tree/surface_normals_for_colormapping) of Open3D. Make sure to get the `surface_normals_for_colormapping` branch. Compile it from source ([instructions](http://www.open3d.org/docs/compilation.html)).
2. Install [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) (the `ros-kinetic-desktop-full` version).
3. `sudo apt-get install python-qt5`
4. `pip install -r requirements.txt`
5. Download and set up [ip_basic](https://github.com/kujason/ip_basic) for depth map hole filling, and add it to the `PYTHONPATH` environment variable.
6. Set up the following repositories in your `catkin_ws/src`:
- https://github.com/ros-perception/camera_info_manager_py
- https://github.com/samarth-robo/mocap_optitrack
- https://github.com/samarth-robo/cv_camera: My fork of the `cv_camera` ROS package with FLIR thermal camera support

Check that `#include <pcl/registration/boost.h>` is included in PCL's header file `ROOT/registration/warp_point_rigid.h`.

## Installing Horus (only needed if you want to record your own data)
We use the [Ciclop 3D scanner from CowTech](https://www.cowtechengineering.com/3d-scanners) for recording data, which is operated by the [Horus](https://horus.readthedocs.io/) software.
1. Remove system opencv: `sudo apt-get remove libopencv-dev && sudo apt-get autoremove`
2. Install dependencies from [here](https://github.com/LibreScanner/horus/blob/develop/doc/development/ubuntu.md)
3. `sudo add-apt-repository ppa:bqlabs/horus && sudo apt-get update`
4. `sudo apt-get install python-opencv` - this installs a modified version of OpenCV for use with Horus. It will not interfere with ROS OpenCV 
6. `sudo apt-get install horus`
7. Permissions: `sudo usermod -a -G dialout $USER` and reboot.
