# netusb_camera_driver
- - -

The ROS Interface for NETUSB Camera with Driver

## Installation

0. Download package

```bash
$ cd /path/to/catkin_ws
$ wstool set netusb_camera_driver --git https://github.com/furushchev/netusb_camera_driver -t src
$ wstool up netusb_camera_driver
```

1. Install dependencies

```bash
$ rosdep install --from-paths src --ignore-src -r -n -y
```

2. Build this package

```bash
$ catkin build netusb_camera_driver
```

2. Install udev rules to recognize camera

```bash
$ roscd netusb_camera_driver
$ sudo cp udev/99-netusbcam.rules /etc/udev/rules.d
```

## Launching Camera Nodes

1. Launch Driver

``` bash
$ roslaunch netusb_camera_driver camera.launch
```

3. Launch Driver, Viewer and Reconfigure

```bash
$ roslaunch netusb_camera_driver sample.launch
```
