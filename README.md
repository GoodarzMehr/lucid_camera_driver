Lucid Vision Labs Camera Driver for CARMA
=========================================
This is a fork of the [arena_camera_ros](https://github.com/lucidvisionlabs/arena_camera_ros) package that is used for connecting to, receiving data from, and configuring [Lucid Vision Labs](https://thinklucid.com/) cameras. This fork has been modified to allow for building a Docker image that can serve as a camera driver for the [CARMA Platform](https://github.com/usdot-fhwa-stol/carma-platform), along with new added functionalities.

Ubuntu 20.04 Installation
-------------------------
Assuming the CARMA Platform is installed at `~/carma_ws/src`,
```
cd ~/carma_ws/src
git clone https://github.com/VT-ASIM-LAB/lucid_camera_driver.git
cd lucid_camera_driver/docker
sudo ./build-image.sh -d
```

Original Arena Camera Driver for ROS1 Documentation
===================================================

Getting Started
---------------
setup and usage https://support.thinklucid.com/using-ros-for-linux/


for ROS2 driver contact LUCID support for more information 
https://support.thinklucid.com/contact-support/
