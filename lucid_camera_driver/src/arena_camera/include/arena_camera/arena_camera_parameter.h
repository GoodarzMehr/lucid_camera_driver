/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef ARENA_CAMERA_ARENA_CAMERA_PARAMETER_H
#define ARENA_CAMERA_ARENA_CAMERA_PARAMETER_H

#include <ros/ros.h>
#include <string>
#include <vector>

namespace arena_camera
{
  enum SHUTTER_MODE
  {
    SM_ROLLING = 0,
    SM_GLOBAL = 1,
    SM_GLOBAL_RESET_RELEASE = 2,
    SM_DEFAULT = -1,
  };

  /**
  *Parameter class for the ArenaCamera
  */
  class ArenaCameraParameter
  {
    public:
      ArenaCameraParameter();

      virtual ~ArenaCameraParameter();

      /**
       * Read the parameters from the parameter server.
       * If invalid parameters are detected, the interface will reset them
       * to their default value.
       * @param nh the ros::NodeHandle to use.
       */
      void readFromRosParameterServer(const ros::NodeHandle& nh);

      /**
       * Gets device_user_id_ from the ROS parameter server.
       */
      const std::string& deviceUserID() const;

      /**
       * Sets device_user_id_ for the class and the ROS parameter server.
       */
      void adaptDeviceUserId(const ros::NodeHandle& nh, const std::string& device_user_id);

      /**
       * Gets the string describing the shutter mode.
       */
      std::string shutterModeString() const;

      /**
       * Gets camera_frame_ from the ROS parameter server.
       */
      const std::string& cameraFrame() const;

      /**
       * Gets the frame_rate_ read from the ROS parameter server.
       */
      const double& frameRate() const;

      /**
       * Gets the image_encoding_ read from the ROS parameter server.
       */
      const std::string& imageEncoding() const;

      /**
       * Sets the frame_rate_ that was initially set from the ROS parameter
       * server. The frame rate needs to be updated with the value the camera
       * supports.
       */
      void setFrameRate(const ros::NodeHandle& nh, const double& frame_rate);

      /**
       * Gets camera_info_url from the ROS parameter server.
       */
      const std::string& cameraInfoURL() const;

      /**
       * Sets camera_info_url_ if a new CameraInfo message object is provided
       * via the SetCameraInfo service of the CameraInfoManager.
       */
      void setCameraInfoURL(const ros::NodeHandle& nh, const std::string& camera_info_url);

    public:
      /** Binning factor to get downsampled images. It refers here to any
       * camera setting which combines rectangular neighborhoods of pixels into
       * larger "super-pixels." It reduces the resolution of the output image
       * to (width / binning_x) x (height / binning_y).
       * The default values of binning_x = binning_y = 0 are considered the
       * same as binning_x = binning_y = 1 (no binning).
       */
      size_t binning_x_;
      size_t binning_y_;

      /**
       * Flags which indicate if binning factors are provided and hence should
       * be set during startup.
       */
      bool binning_x_given_;
      bool binning_y_given_;

      /**
       * Flag which indicates whether sensor or digital binning should be used.
       */
      bool sensor_binning_;

      /**
       * Flag which indicates if a desired image pixel encoding is provided.
       */
      bool image_encoding_given_;

      // ######################################################################
      // ###################### Image Intensity Settings  #####################
      // ######################################################################
      // The following settings do *NOT* have to be set. Each camera has
      // default values that provide automatic image adjustment.
      // #######################################################################

      /**
       * The desired exposure time in microseconds.
       */
      double exposure_;

      /**
       * Flag which indicates if exposure time is provided and hence should
       * be set during startup.
       */
      bool exposure_given_;

      /**
       * The target gain as a percent of the maximum value that the camera
       * supports. For USB cameras, the gain is in dB, for GigE cameras it is
       * given in so-called 'device specific units'.
       */
      double gain_;

      /**
       * Flag which indicates if gain is provided and hence should be set
       * during startup.
       */
      bool gain_given_;

      /**
       * Gamma correction of pixel intensity.
       * Adjusts the brightness of the pixel values output by the camera to
       * account for the non-linearity in the human perception of brightness or
       * that of the display system (such as CRT).
       */
      double gamma_;

      /**
       * Flag which indicates if gamma correction is provided and hence should
       * be set during startup.
       */
      bool gamma_given_;

      /**
       * The average intensity value of the image. It depends on both the
       * exposure time and gain. If 'exposure time' is provided, the interface
       * will try to reach the desired brightness by only varying the gain (It
       * may often fail, because the range of possible exposure times is much
       * greater than the possible gain range). If 'gain' is provided, the
       * interface will try to reach the desired brightness by only varying the
       * exposure time. If gain AND exposure time are both given, it will not
       * be possible to reach the desired brightness.
       */
      int brightness_;

      /**
       * Flag which indicates if brightness is provided and hence should be
       * set during startup
       */
      bool brightness_given_;

      /**
       * Only relevant if 'brightness' is given as a ROS parameter. If the
       * camera should try to reach and/or maintain the desired brightness,
       * i.e. adapting to the changing light conditions, at least one of the
       * following flags must be set. If both are set, the interface will use
       * the profile that tries to keep the gain at a minimum to reduce white
       * noise. The exposure_auto_ flag indicates that the desired brightness
       * may be reached by changing the exposure time. The gain_auto_ flag
       * indicates that the desired brightness may be reached by changing the
       * gain.
       */
      bool exposure_auto_;
      bool gain_auto_;
      // ######################################################################

      /**
       * MTU size. Only used for GigE cameras.
       * To prevent lost frames the camera has to be configured with the MTU
       * size that the network card supports. A value greater than 3000 should
       * work in most cases (1500 for Raspberry Pi).
       */
      int mtu_size_;

      /**
       * Inter-package delay in ticks. Only used for GigE cameras.
       * To prevent lost frames it should be greater than 0.
       * For most GigE cameras, a value of 1000 is reasonable.
       * For GigE cameras used with a Raspberry Pi this value should be set to
       * 11772.
       */
      int inter_pkg_delay_;

      /**
       * The shutter mode used by the camera.
       */
      SHUTTER_MODE shutter_mode_;

      /**
       * Flag that indicates if the camera has been calibrated and the
       * calibration matrices have been provided.
       */
      bool has_intrinsic_calib_;

      /**
       * Flag that indicates if the camera has a flash connected which should
       * turn on on exposure. It is set to false by default.
       * Only supported for GigE cameras.
       */
      bool auto_flash_;
      
      /**
       * Flag that indicates if the camera is using auto flash and has a flash
       * connected on line 2 which should turn on on exposure. It is set to
       * false by default.
       * Only supported for GigE cameras.
       */
      bool auto_flash_line_2_;
      
      /**
       * Flag that indicates if the camera is using auto flash and has a flash
       * connected on line 3 which should turn on on exposure. It is set to
       * false by default.
       * Only supported for GigE cameras.
       */
      bool auto_flash_line_3_;

    protected:
      /**
       * Validates the parameter set found on the ROS parameter server. If
       * invalid parameters are detected, the interface will reset them to
       * their default values.
       * @param nh the ros::NodeHandle to use.
       */
      void validateParameterSet(const ros::NodeHandle& nh);

      /**
       * The TF2 frame under which the images are published.
       */
      std::string camera_frame_;

      /**
       * The DeviceUserID of the camera. If empty, the first camera found in
       * the device list will be used.
       */
      std::string device_user_id_;

      /**
       * The desired camera frame rate. It can only be set once at startup.
       * Calling the GrabImages action can result in a higher frame rate.
       */
      double frame_rate_;

      /**
       * The CameraInfo URL (Uniform Resource Locator) where the optional
       * intrinsic camera calibration parameters are stored. This URL string
       * will be parsed using the CameraInfoManager.
       * http://wiki.ros.org/camera_info_manager
       */
      std::string camera_info_url_;

      /**
       * The image pixel encoding (channel meaning, ordering, size) taken
       * from the list of strings in include/sensor_msgs/image_encodings.h.
       * The supported encodings are 'mono8', 'mono16', 'bgr8', 'bgr16',
       * 'bgr24', 'rgb8', 'rgb16', 'rgb24', 'bayer_bggr8', 'bayer_bggr16',
       * 'bayer_bggr24', 'bayer_gbrg8', 'bayer_gbrg16', 'bayer_gbrg24',
       * 'bayer_rggb8', 'bayer_rggb16', 'bayer_rggb24', and 'yuv422'.
       */
      std::string image_encoding_;
  };
}  // namespace arena_camera

#endif  // ARENA_CAMERA_ARENA_CAMERA_PARAMETER_H