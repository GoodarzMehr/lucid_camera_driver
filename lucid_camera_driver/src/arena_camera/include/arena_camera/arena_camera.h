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

#ifndef ARENA_CAMERA_ARENA_CAMERA_H
#define ARENA_CAMERA_ARENA_CAMERA_H

#include <map>
#include <string>
#include <vector>

#include <arena_camera/arena_camera_parameter.h>
#include <sensor_msgs/RegionOfInterest.h>

namespace arena_camera
{
  /**
   * The ArenaCamera base class. Create a new instance using the static
   * create() function.
   */
  class ArenaCamera
  {
    public:
      /**
       * Creates a new ArenaCamera instance based on the DeviceUserID of the
       * camera.
       * @param device_user_id Arena DeviceUserID. If the string is empty, the
       * first camera that could be found is returned.
       * @return new ArenaCamera instance or NULL if the camera was not found.
       */
      static ArenaCamera* create(const std::string& device_user_id);

      /**
       * Configures the camera according to the software trigger mode.
       * @return true if all configurations could be set up.
       */
      virtual bool registerCameraConfiguration() = 0;

      /**
       * Opens the desired camera. Communication starts from now on.
       * @return true if the camera could be opened.
       */
      virtual bool openCamera() = 0;

      /**
       * Returns the connection state of the camera.
       * @return true if camera removal from the PC has been detected.
       */
      virtual bool isCamRemoved() = 0;

      /**
       * Configures the sequencer exposure times.
       * @param exposure_times the list of exposure times.
       * @return true if all parameters could be sent to the camera.
       */
      virtual bool setupSequencer(const std::vector<float>& exposure_times) = 0;

      /**
       * Configures the camera according to the provided ROS parameters.
       * This will use device-specific parameters such as MTU size for GigE
       * cameras.
       * @param parameters The ArenaCameraParameter set to use.
       * @return true if all parameters could be sent to the camera.
       */
      virtual bool applyCamSpecificStartupSettings(const ArenaCameraParameter& parameters) = 0;

      /**
       * Initializes the internal parameters of the ArenaCamera instance.
       * @param parameters The ArenaCameraParameter set to use.
       * @return true if all parameters could be sent to the camera.
       */
      virtual bool startGrabbing(const ArenaCameraParameter& parameters) = 0;

      /**
       * Grabs a camera frame and copy the result into image.
       * @param image reference to the output image.
       * @return true if the image was grabbed successfully.
       */
      virtual bool grab(std::vector<uint8_t>& image) = 0;

      /**
       * Grabs a camera frame and copy the result into image.
       * @param image pointer to the image buffer.
       *              Caution: Make sure the buffer is initialized correctly!
       * @return true if the image was grabbed successfully.
       */
      virtual bool grab(uint8_t* image) = 0;

      /**
       * Sets the shutter mode for the camera (rolling or global).
       * @param mode shutter mode.
       * @return true if shutter mode could be set.
       */
      virtual bool setShutterMode(const arena_camera::SHUTTER_MODE& mode) = 0;

      /**
       * Updates area of interest in the camera image.
       * @param target_roi the target ROI.
       * @param reached_roi the ROI that could be set.
       * @return true if the target ROI could be reached.
       */
      virtual bool setROI(const sensor_msgs::RegionOfInterest target_roi,
                          sensor_msgs::RegionOfInterest& reached_roi) = 0;

      /**
       * Sets the target horizontal binning factor.
       * @param target_binning_x the target horizontal binning factor.
       * @param reached_binning_x the reached horizontal binning factor.
       * @return true if the target horizontal binning factor could be reached.
       */
      virtual bool setBinningX(const size_t& target_binning_x,
                                size_t& reached_binning_x) = 0;

      /**
       * Sets the target vertical binning factor.
       * @param target_binning_y the target vertical binning factor.
       * @param reached_binning_y the reached vertical binning factor.
       * @return true if the target vertical binning factor could be reached.
       */
      virtual bool setBinningY(const size_t& target_binning_y,
                                size_t& reached_binning_y) = 0;

      /**
       * Detects supported image pixel encodings of the camera and stores them
       * in a vector.
       * @return a list of strings describing the supported encodings in GenAPI
       *         language.
       */
      virtual std::vector<std::string> detectAvailableImageEncodings() = 0;

      /**
       * Sets the desired image pixel encoding
       * (channel meaning, ordering, size) taken from the list of strings in
       * include/sensor_msgs/image_encodings.h.
       * @param target_ros_encoding: string describing the encoding.
       * @return true if the desired image encoding could be set.
       */
      virtual bool setImageEncoding(const std::string& target_ros_encoding) = 0;

      /**
       * Sets the exposure time in microseconds.
       * @param target_exposure the desired exposure time in microseconds.
       * @param reached_exposure the reached exposure time in microseconds.
       * @return true if the desired exposure time could be set.
       */
      virtual bool setExposure(const float& target_exposure, float& reached_exposure) = 0;

      /**
       * Sets Autoflash active for the specified lines.
       * @param flash_on_lines map from line 1 or 2 to a boolean to activate or
       * deactivate the Autoflash for that line.
       * @return true if Autoflash could be set to active for the specified
       * lines.
       */
      virtual bool setAutoflash(const std::map<int, bool> flash_on_lines) = 0;
      
      /**
       * Sets the gain in percent independent of camera type.
       * @param target_gain the target gain in percent.
       * @param reached_gain the reached gain in percent.
       * @return true if the target gain could be set.
       */
      virtual bool setGain(const float& target_gain, float& reached_gain) = 0;

      /**
       * Sets the target gamma value.
       * @param target_gamma the target gamma value.
       * @param reached_gamma the reached gamma value.
       * @return true if the target gamma value could be set.
       */
      virtual bool setGamma(const float& target_gamma,
                            float& reached_gamma) = 0;

      /**
       * Sets the target brightness.
       * @param target_brightness the desired brightness, in the range
       * [1...255].
       * @param current_brightness the current brightness.
       * @param exposure_auto flag which indicates if the target brightness
       * should be reached by changing the exposure time.
       * @param gain_auto flag which indicates if the target brightness should
       * be reached by changing the gain.
       * @return true if the target brightness could be reached.
       */
      virtual bool setBrightness(const int& target_brightness,
                                  const float& current_brightness,
                                  const bool& exposure_auto,
                                  const bool& gain_auto) = 0;

      /**
       * Detects and counts the number of user-settable outputs the camera
       * provides. This might be zero for some cameras. The size affects the
       * number of 'set' ROS services the camera node will provide. A vector
       * whose length equals the number of user-settable outputs will be
       * generated. Hence e.g. output '1' can be accessed via
       * user_output_selector_enums_.at(1).
       * @return the UserOutputSelector enumeration list.
       */
      virtual std::vector<int> detectAndCountNumUserOutputs() = 0;

      /**
       * Sets the user output.
       * @param output_id ID of the output to be set.
       * @param value desired value for the output.
       * @return true if the desired output value was set.
       */
      virtual bool setUserOutput(const int& output_id, const bool& value) = 0;

      /**
       * Returns the current X offset value.
       * @return the current X offset value.
       */
      virtual size_t currentOffsetX() = 0;

      /**
       * Returns the current Y offset value.
       * @return the current Y offset value.
       */
      virtual size_t currentOffsetY() = 0;

      /**
       * Returns the current ROI.
       * @return the current ROI.
       */
      virtual sensor_msgs::RegionOfInterest currentROI() = 0;

      /**
       * Returns the current horizontal binning value.
       * @return the current horizontal binning value.
       */
      virtual size_t currentBinningX() = 0;

      /**
       * Returns the current vertical binning value.
       * @return the current vertical binning value.
       */
      virtual size_t currentBinningY() = 0;

      /**
       * Gets the current image pixel encoding from the ones listed in
       * sensor_msgs::image_encodings.
       * @return the current ROS image pixel encoding.
       */
      virtual std::string currentROSEncoding() const = 0;

      /**
       * Gets the number of bits per pixel.
       * @return the number of bits per pixel.
       */
      virtual int imagePixelDepth() const = 0;

      /**
       * Returns the current exposure time in microseconds.
       * @return the current exposure time in microseconds.
       */
      virtual float currentExposure() = 0;

      /**
       * Returns the current auto exposure time lower limit.
       * @return the current auto exposure time lower limit.
       */
      virtual float currentAutoExposureTimeLowerLimit() = 0;

      /**
       * Returns the current auto exposure time upper limit.
       * @return the current auto exposure time upper limit.
       */
      virtual float currentAutoExposureTimeUpperLimit() = 0;

      /**
       * Returns the current gain in percent.
       * @return the current gain in percent.
       */
      virtual float currentGain() = 0;

      /**
       * Returns the current auto gain lower limit.
       * @return the current auto gain lower limit.
       */
      virtual float currentAutoGainLowerLimit() = 0;

      /**
       * Returns the current auto gain upper limit.
       * @return the current auto gain upper limit.
       */
      virtual float currentAutoGainUpperLimit() = 0;
      
      /**
       * Returns the current gamma value.
       * @return the current gamma value.
       */
      virtual float currentGamma() = 0;

      /**
       * Checks if the camera is currently trying to regulate towards a target
       * brightness. This can either be done by Arena for the range [50 - 205]
       * or a binary search one for ranges [1 - 49] and [206 - 254].
       * @return true if the brightness search is running.
       */
      virtual bool isBrightnessSearchRunning() = 0;

      /**
       * Checks if the AutoBrightness function from the Arena API is enabled.
       * @return true if AutoExposure is set to AutoExposureContinuous or
       * AutoExposureOnce.
       */
      virtual bool isArenaAutoBrightnessFunctionRunning() = 0;

      /**
       * Disables all currently running brightness search methods in case the
       * desired brightness is reached or a timeout has occurred.
       */
      virtual void disableAllRunningAutoBrightessFunctions() = 0;

      /**
       * Enables the Continuous auto exposure mode.
       */
      virtual void enableContinuousAutoExposure() = 0;

      /**
       * Enables the Continuous auto gain mode.
       */
      virtual void enableContinuousAutoGain() = 0;

      /**
       * Gets the camera type. Currently supported cameras are USB, DART and
       * GigE.
       * @return camera type as a string.
       */
      virtual std::string typeName() const = 0;

      /**
       * Gets the exposure step, or the minimum possible increment between two
       * possible exposure values.
       * @return the exposure step.
       */
      virtual float exposureStep() = 0;

      /**
       * Gets the device user ID of the camera.
       * @return the device_user_id.
       */
      const std::string& deviceUserID() const;

      /**
       * Gets the image height.
       * @return number of rows in the image.
       */
      const size_t& imageRows() const;

      /**
       * Gets the image width
       * @return number of columns in the image.
       */
      const size_t& imageCols() const;

      /**
       * Gets the is_ready_ flag. This is set in case the grab-result pointer
       * of the first acquisition contains valid data. Hence, this is the
       * current state of the interface.
       * @return true if the interface is ready.
       */
      const bool& isReady() const;

      /**
       * Returns the number of digital user outputs, which can be set by the
       * camera. Might be zero for some cameras. The size affects the number of
       * 'set' ROS services the camera node will provide.
       * @return number of digital user outputs.
       */
      std::size_t numUserOutputs() const;

      /**
       * Returns the image size in bytes.
       * @return the image size in bytes.
       */
      const size_t& imageSize() const;

      /**
       * Gets the maximum achievable frame rate.
       * @return the maximum achievable frame rate.
       */
      virtual float maxPossibleFramerate() = 0;

      /**
       * Checks if the camera has the auto exposure feature.
       * @return true if the camera supports auto exposure.
       */
      const bool& hasAutoExposure() const;

      /**
       * Maximum difference allowed between target and reached brightness values.
       * @return the allowed tolerance.
       */
      const float& maxBrightnessTolerance() const;

      /**
       * Gets the sequencer exposure times.
       * @return the list of exposure times
       */
      const std::vector<float>& sequencerExposureTimes() const;

      virtual ~ArenaCamera();

    protected:
      /**
       * Protected default constructor.
       */
      ArenaCamera();

      /**
       * Enables the extended brightness search.
       * @param brightness target brightness
       * @return true if the target brightness could be reached.
       */
      virtual bool setExtendedBrightness(const int& target_brightness,
                                          const float& current_brightness) = 0;

      /**
       * The DeviceUserID of the camera.
       */
      std::string device_user_id_;

      /**
       * Number of image rows.
       */
      size_t img_rows_;

      /**
       * Number of image columns.
       */
      size_t img_cols_;

      /**
       * The size of the image in bytes.
       */
      size_t img_size_byte_;

      /**
       * The maximum time allowed for a single image grab. This value should
       * always be greater than the maximum possible exposure time of the
       * camera.
       */
      float grab_timeout_;

      /**
       * Flag which is set in case the grab-result pointer of the first
       * acquisition contains valid data.
       */
      bool is_ready_;

      /**
       * Maximum allowed difference between target and reached brightness
       * values.
       */
      const float max_brightness_tolerance_;

      /**
       * Exposure times to use when in sequencer mode.
       */
      std::vector<float> seq_exp_times_;

      /**
       * Vector containing all available user outputs.
       */
      std::vector<int> user_output_selector_enums_;

      /**
       * Vector that contains the available image encodings the camera
       * supports. The strings describe the GenAPI encodings.
       */
      std::vector<std::string> available_image_encodings_;
  };
}  // namespace arena_camera

#endif  // ARENA_CAMERA_ARENA_CAMERA_H