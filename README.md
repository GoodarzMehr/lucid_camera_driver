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
* `arena_camera_node/image_scaled [sensor_msgs/Image]`: publishes a demosaiced RGB image if raw images have Bayer encoding. Furthermore, since RViz cannot display images with a bit-depth higher than 8, if raw images have a bit-depth of 16 or 24 they are converted to an 8-bit image and published by this topic. In all other cases (8-bit RGB, BGR, or Mono image) it publishes the same image as the `arena_camera_node/image_raw` topic (20 Hz).
* `arena_camera_node/camera_info [sensor_msgs/CameraInfo]`: publishes the [camera calibration file](http://www.ros.org/wiki/camera_calibration_parsers#File_formats) (20 Hz).
* `arena_camera_node/discovery [cav_msgs/DriverStatus]`: publishes the CARMA [DriverStatus](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/cav_msgs/msg/DriverStatus.msg) message (1.25 Hz).
* `diagnostics [diagnostic_msgs/DiagnosticArray]`: publishes diagnostic information about the state of the camera (0.5 Hz).

#### Subscribed Topics
N/A

#### Services
* `arena_camera_node/get_properties [camera_control_msgs/GetCamProperties]`: gets a list of current camera properties, including information about the camera device user ID, horizontal and vertical binning factors, frame rate, exposure time, gain, gamma correction, brightness, whether auto gain and auto exposure functions are enabled, and whether the camera is sleeping.
* `arena_camera_node/set_binning [camera_control_msgs/SetBinning]`: configures the horizontal and vertical binning factors of the camera.
* `arena_camera_node/set_brightness [camera_control_msgs/SetBrightness]`: configures the target brightness of the camera. The user can specify whether the target brightness should be reached by changing exposure time, gain, or both.
* `arena_camera_node/set_camera_info [sensor_msgs/SetCameraInfo]`: stores the given CameraInfo as the camera's calibration information.
* `arena_camera_node/set_exposure [camera_control_msgs/SetExposure]`: configures the exposure time of the camera.
* `arena_camera_node/set_gain [camera_control_msgs/SetGain]`: configures the analog gain of the camera.
* `arena_camera_node/set_gamma [camera_control_msgs/SetGamma]`: configures the gamma correction factor of the camera.
* `arena_camera_node/set_roi [camera_control_msgs/SetROI]`: configures the region of interest (RoI) of the camera.
* `arena_camera_node/set_sleeping [camera_control_msgs/SetSleeping]`: puts the camera to or wakes it up from sleep.

#### Parameters
* `arena_camera_node/camera_frame`: the [TF2](http://www.ros.org/wiki/tf2) frame ID of the camera. It is set to `arena_camera` by default.
* `arena_camera_node/device_user_id`: the device user ID of the camera. If empty, the first camera found in the device list will be used.
* `arena_camera_node/frame_rate`: the frame rate of the camera. It is set to 10.0 Hz by default.
* `arena_camera_node/camera_info_url`: the path to the [camera calibration file](http://www.ros.org/wiki/camera_calibration_parsers#File_formats).
* `arena_camera_node/image_encoding`: the image pixel encoding of the camera, which defines the number and order of color channels and pixel bit-depth. It can be any of the following, if supported by the camera: `rgb8`, `rgb16`, `rgb24`, `bgr8`, `bgr16`, `bgr24`, `bayer_rggb8`, `bayer_rggb16`, `bayer_rggb24`, `bayer_bggr8`, `bayer_bggr16`, `bayer_bggr24`, `bayer_grbg8`, `bayer_grbg16`, `bayer_grbg24`, `mono8`, `mono16`.
* `arena_camera_node/binning_x`: the horizontal binning factor of the camera. It is set to 1 by default.
* `arena_camera_node/binning_y`: the vertical binning factor of the camera. It is set to 1 by default.
* `arena_camera_node/sensor_binning`: specifies whether sensor binning or digital binning is used. It is set to true (sensor binning) by default.
* `arena_camera_node/exposure`: the exposure time of the camera, given in microseconds. It is set to 100000 microseconds by default.
* `arena_camera_node/gain`: the analog gain of the camera, given as a fraction of the maximum possible value. It is set to 0.5 by default.
* `arena_camera_node/gamma`: the gamma correction value of the camera. It adjusts the brightness of the pixel values output by the camera to account for the non-linearity in the human perception of brightness or that of the display system (e.g. a CRT display). It is set to 1.0 by default.
* `arena_camera_node/brightness`: the target brightness of the camera, which is a value in the [1, 255] interval. It is set to 70 by default.
* `arena_camera_node/exposure_auto`: specifies whether auto exposure is turned on. It is set to true (auto exposure on) by default.
* `arena_camera_node/gain_auto`: specifies whether auto gain is turned on. It is set to true (auto gain on) by default.
* `arena_camera_node/mtu_size`: the maximum transmission unit (MTU) size of the camera (only for cameras using the GigE Vision interface). It is set to 9000 by default.
* `arena_camera_node/inter_pkg_delay`: the inter-package delay of the camera, given in ticks (only for cameras using the GigE Vision interface). It is set to 1000 by default.
* `arena_camera_node/shutter_mode`: the shutter mode of the camera (global or rolling), if the camera supports different shutter modes.
* `arena_camera_node/auto_flash`: specifies whether auto flash is turned on. It is set to false (auto flash off) by default.

Examples
--------
See the `lucid_single.launch` and `lucid_multi.launch` files in the `arena_camera/launch` directory that are used to launch one and multiple cameras, respectively.

Original Arena Camera Driver for ROS1 Documentation
===================================================

Getting Started
---------------
Setup and usage https://support.thinklucid.com/using-ros-for-linux/


For ROS2 driver contact LUCID support for more information 
https://support.thinklucid.com/contact-support/
