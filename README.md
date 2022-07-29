Lucid Vision Labs Camera Driver for CARMA
=========================================
This is a fork of the [arena_camera_ros](https://github.com/lucidvisionlabs/arena_camera_ros) package that is used for connecting to, receiving data from, and configuring [Lucid Vision Labs](https://thinklucid.com/) cameras. This fork has been modified to allow for building a Docker image that can serve as a camera driver for the [CARMA Platform](https://github.com/usdot-fhwa-stol/carma-platform), along with new functionalities.

Ubuntu 20.04 Installation
-------------------------
Assuming the CARMA Platform is installed at `~/carma_ws/src`,
```
cd ~/carma_ws/src
git clone https://github.com/VT-ASIM-LAB/lucid_camera_driver.git
cd lucid_camera_driver/docker
sudo ./build-image.sh -d
```
After the docker image is successfully built, add the following lines to the appropriate `docker-compose.yml` file in the `carma-config` directory.
```
lucid-camera-driver:
  image: usdotfhwastoldev/carma-lucid-camera-driver:develop
  container_name: lucid-camera-driver
  network_mode: host
  volumes_from:
    - container:carma-config:ro
  environment:
    - ROS_IP=127.0.0.1
  volumes:
    - /opt/carma/logs:/opt/carma/logs
    - /opt/carma/.ros:/home/carma/.ros
    - /opt/carma/vehicle/calibration:/opt/carma/vehicle/calibration
  command: bash -c '. ./devel/setup.bash && export ROS_NAMESPACE=$${CARMA_INTR_NS} && wait-for-it.sh localhost:11311 -- roslaunch /opt/carma/vehicle/config/drivers.launch drivers:=lucid_camera'
```
Finally, add the following lines to the `drivers.launch` file in the same directory as `docker-compose.yml`.
```
<include if="$(arg lucid_camera)" file="$(find arena_camera)/launch/lucid_single.launch">
</include>
```

ROS API
-------

### arena_camera

#### Nodes
* `arena_camera_node`
* `write_device_user_id_to_camera`

#### Published Topics
Publication frequencies are given for a [Lucid Vision Labs TRI054S-CC](https://thinklucid.com/product/triton-5-mp-imx490/) camera operating at full resolution (2880x1860) with a binning factor of 2 (effective resolution of 1440x930), default image encoding (24-bit Bayer RGGB), and a frame rate of 20 Hz.
* `arena_camera_node/image_raw [sensor_msgs/Image]`: publishes raw images obtained from the camera (20 Hz).
* `arena_camera_node/image_rect [sensor_msgs/Image]`: publishes rectified raw images obtained from the camera only if a [camera calibration file](https://wiki.ros.org/camera_calibration_parsers#File_formats) has been provided (20 Hz).
* `arena_camera_node/image_scaled [sensor_msgs/Image]`: publishes a demosaiced RGB image if raw images have Bayer encoding. Furthermore, since RViz cannot display images with a bit-depth higher than 8, if raw images have a bit-depth of 16 or 24 they are converted to an 8-bit image and published by this topic (20 Hz).
* `arena_camera_node/camera_info [sensor_msgs/CameraInfo]`: publishes the [camera calibration file](http://www.ros.org/wiki/camera_calibration_parsers#File_formats) (30 Hz).
* `arena_camera_node/discovery [cav_msgs/DriverStatus]`: publishes the CARMA [DriverStatus](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/cav_msgs/msg/DriverStatus.msg) message (1.25 Hz).
* `diagnostics [diagnostic_msgs/DiagnosticArray]`: publishes diagnostic information about the state of the camera (0.5 Hz).

#### Subscribed Topics
N/A

#### Services
* `arena_camera_node/get_properties [camera_control_msgs/GetCamProperties]`: gets a list of current camera properties, including information about whether the camera device user ID, horizontal and vertical binning factors, frame rate, exposure time, gain, gamma correction, brightness, whether auto gain and auto exposure functions are enabled, and whether the camera is sleeping.
* `arena_camera_node/set_binning [camera_control_msgs/SetBinning]`: configures the horizontal and vertical binning factors of the camera.
* `arena_camera_node/set_brightness [camera_control_msgs/SetBrightness]`: configures the target brightness of camera. The user can specify whether the target brightness should be reached by chaning exposure time, gain, or both.
* `arena_cmaera_node/set_camera_info [sensor_msgs/SetCameraInfo]`: stores the given CameraInfo as the camera's calibration information.

Original Arena Camera Driver for ROS1 Documentation
===================================================

Getting Started
---------------
setup and usage https://support.thinklucid.com/using-ros-for-linux/


for ROS2 driver contact LUCID support for more information 
https://support.thinklucid.com/contact-support/
