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

// STD
#include <ios>

// ROS
#include <sensor_msgs/image_encodings.h>

// Arena node
#include <arena_camera/arena_camera_parameter.h>

namespace arena_camera
{
  ArenaCameraParameter::ArenaCameraParameter()
    : camera_frame_("arena_camera")
    , device_user_id_("")
    , frame_rate_(10.0)
    , camera_info_url_("")
    , image_encoding_("")
    , image_encoding_given_(false)
    , binning_x_(1)
    , binning_y_(1)
    , binning_x_given_(false)
    , binning_y_given_(false)
    , sensor_binning_(true)
    , exposure_(100000.0)
    , exposure_given_(false)
    , gain_(0.5)
    , gain_given_(false)
    , gamma_(1.0)
    , gamma_given_(false)
    , brightness_(70)
    , brightness_given_(false)
    , exposure_auto_(true)
    , gain_auto_(true)
    , mtu_size_(9000)
    , inter_pkg_delay_(1000)
    , shutter_mode_(SM_DEFAULT)
    , auto_flash_(false)
  {
  }

  ArenaCameraParameter::~ArenaCameraParameter()
  {
  }

  void ArenaCameraParameter::readFromRosParameterServer(const ros::NodeHandle& nh)
  {
    // Define TF2 frame ID string.
    nh.param<std::string>("camera_frame", camera_frame_, "arena_camera");

    // Define device user ID string.
    nh.param<std::string>("device_user_id", device_user_id_, "");

    // Get frame rate if it has been provided.
    if (nh.hasParam("frame_rate"))
    {
      nh.getParam("frame_rate", frame_rate_);
      
      ROS_DEBUG_STREAM("Frame rate has been given and has a value of " << frame_rate_ << " Hz.");
    }

    // Define camera info URL string and get it if it has been provided.
    nh.param<std::string>("camera_info_url", camera_info_url_, "");
    
    if (nh.hasParam("camera_info_url"))
    {
      nh.getParam("camera_info_url", camera_info_url_);
    }

    // Define sensor binning flag and get it if it has been provided.
    nh.param<bool>("sensor_binning", sensor_binning_, true);

    if (nh.hasParam("sensor_binning"))
    {
      nh.getParam("sensor_binning", sensor_binning_);
    }

    // Check if horizontal binning factor has been provided. If so, check its
    // validity.
    binning_x_given_ = nh.hasParam("binning_x");
    
    if (binning_x_given_)
    {
      int binning_x;
      
      nh.getParam("binning_x", binning_x);
      
      ROS_DEBUG_STREAM( "Horizontal binning factor has been given and has a value of "
                        << binning_x << ".");
      
      if (binning_x > 32 || binning_x < 0)
      {
        ROS_WARN_STREAM("Desired horizontal binning factor (" << binning_x << ") is invalid. It "
                        << "will be reset to the default value (1).");
        
        binning_x_given_ = false;
      }
      else
      {
        binning_x_ = static_cast<size_t>(binning_x);
      }
    }

    // Check if vertical binning factor has been provided. If so, check its
    // validity.
    binning_y_given_ = nh.hasParam("binning_y");
    
    if (binning_y_given_)
    {
      int binning_y;
      
      nh.getParam("binning_y", binning_y);
      
      ROS_DEBUG_STREAM( "Vertical binning factor has been given and has a value of " << binning_y
                        << ".");
      
      if (binning_y > 32 || binning_y < 0)
      {
        ROS_WARN_STREAM("Desired vertical binning factor (" << binning_y << ") is invalid. It "
                        << "will be reset to the default value (1).");
        
        binning_y_given_ = false;
      }
      else
      {
        binning_y_ = static_cast<size_t>(binning_y);
      }
    }

    // Check if image encoding has been provided. If so, use it.
    image_encoding_given_ = nh.hasParam("image_encoding");
    
    if (image_encoding_given_)
    {
      nh.getParam("image_encoding", image_encoding_);
    }

    //Check if gamma correction has been provided. If so, use it.
    gamma_given_ = nh.hasParam("gamma");
    
    if (gamma_given_)
    {
      nh.getParam("gamma", gamma_);
      
      ROS_DEBUG_STREAM("Gamma has been given and has a value of " << gamma_ << ".");
    }

    // Check if exposure time has been provided. If so, use it.
    exposure_given_ = nh.hasParam("exposure");

    if (exposure_given_)
    {
      nh.getParam("exposure", exposure_);
      
      ROS_DEBUG_STREAM("Exposure time has been given and has a value of " << exposure_ << " us.");
    }

    // Check if gain has been provided. If so, use it.
    gain_given_ = nh.hasParam("gain");

    if (gain_given_)
    {
      nh.getParam("gain", gain_);
      
      ROS_DEBUG_STREAM("Gain fraction has been given and has a value of " << gain_ << ".");
    }

    // Check if brightness has been provided. If so, use it.
    brightness_given_ = nh.hasParam("brightness");

    if (brightness_given_)
    {
      nh.getParam("brightness", brightness_);
      
      ROS_DEBUG_STREAM("Brightness has been given and has a value of " << brightness_ << ".");
    }

    // Ignore brightness if both exposure time and gain have been provided.
    auto ignoreBrightness = brightness_given_ && gain_given_ && exposure_given_;
    
    if (ignoreBrightness)
    {
      ROS_WARN_STREAM("Gain ('gain') and exposure time ('exposure') are given as ROS parameters "
                      << "at startup and assumed to be fixed. Hence the desired brightness ("
                      << brightness_ << ") cannot be reached. Brightness will be ignored.");
      
      brightness_given_ = false;
    }
    
    // Ignore auto exposure if exposure time has been provided.
    /*
      EG := exposure_given
      EA := exposure_auto
      EAG := exposure_auto_given_
      EARV := exposure_auto_ received value

      |-----------------------------------------------------------------------|
      | EG | EAG | EARV | Action                                              |
      |----|-----|------|-----------------------------------------------------|
      | F  | F   | F    | use default values                                  |
      |    |     |      |                                                     |
      | F  | F   | T    | default case, do nothing                            |
      |    |     |      |                                                     |
      | F  | T   | F    | show message and set EA to false in node map        |
      |    |     |      |                                                     |
      | F  | T   | T    | show message                                        |
      |    |     |      |                                                     |
      | T  | F   | F    | use the provided exposure time                      |
      |    |     |      |                                                     |
      | T  | F   | T    | set EA to false, use the provided exposure time     |
      |    |     |      |                                                     |
      | T  | T   | F    | show message                                        |
      |    |     |      |                                                     |
      | T  | T   | T    | ignore EARV and set EA to false, show message       |
      |-----------------------------------------------------------------------|
    */
    
    auto exposure_auto_given = nh.hasParam("exposure_auto");
    
    nh.getParam("exposure_auto", exposure_auto_);
    
    // FFF
    // Default value of auto exposure is used.
    // FFT
    // Default case, nothing done/shown.
    if (!exposure_given_ && !exposure_auto_given && exposure_auto_)
    {
    }
    // FTF
    // Auto exposure is turned off. Exposure time will be read from the device.
    else if(!exposure_given_ && exposure_auto_given && !exposure_auto_)
    {
      ROS_DEBUG_STREAM("exposure_auto is given and set to false (Off)." );
    }
    // FTT
    // Auto exposure is turned on.
    else if(!exposure_given_ && exposure_auto_given && exposure_auto_)
    {
      ROS_DEBUG_STREAM("exposure_auto is given and set to true (Continuous).");
    }
    // TFF
    // The provided exposure time is used.
    // TFT
    // Turn auto exposure off, the provided exposure time is used.
    else if(exposure_given_ && !exposure_auto_given && exposure_auto_)
    {
      exposure_auto_ = false;
    }
    // TTF
    // Auto exposure is turned off, the provided exposure time is used.
    else if(exposure_given_ && exposure_auto_given && !exposure_auto_)
    {
      ROS_DEBUG_STREAM("exposure_auto is given and set to false (Off)." );
    }
    // TTT
    // Turn auto exposure off, the provided exposure time is used.
    else if(exposure_given_ && exposure_auto_given && exposure_auto_)
    {
      exposure_auto_ = false;
      
      ROS_WARN_STREAM("exposure_auto is ignored because exposure is given.");
    }

    // Ignore auto gain if gain has been provided.
    /*
      GG := gain_given
      GA := gain_auto
      GAG := gain_auto_given_
      GARV := gain_auto_ received value

      |-----------------------------------------------------------------------|
      | GG | GAG | GARV | Action                                              |
      |----|-----|------|-----------------------------------------------------|
      | F  | F   | F    | use default values                                  |
      |    |     |      |                                                     |
      | F  | F   | T    | default case, do nothing                            |
      |    |     |      |                                                     |
      | F  | T   | F    | show message and set GA to false in node map        |
      |    |     |      |                                                     |
      | F  | T   | T    | show message                                        |
      |    |     |      |                                                     |
      | T  | F   | F    | use the provided gain                               |
      |    |     |      |                                                     |
      | T  | F   | T    | set GA to false, use the provided gain              |
      |    |     |      |                                                     |
      | T  | T   | F    | show message                                        |
      |    |     |      |                                                     |
      | T  | T   | T    | ignore GARV and set GA to false, show message       |
      |-----------------------------------------------------------------------|
    */
    
    auto gain_auto_given = nh.hasParam("gain_auto");
    
    nh.getParam("gain_auto", gain_auto_);
    
    // FFF
    // Default value of auto gain is used.
    // FFT
    // Default case, nothing done/shown.
    if (!gain_given_ && !gain_auto_given && gain_auto_)
    {
    }
    // FTF
    // Auto gain is turned off. Gain will be read from the device.
    else if(!gain_given_ && gain_auto_given && !gain_auto_)
    {
      ROS_DEBUG_STREAM("gain_auto is given and set to false (Off)." );
    }
    // FTT
    // Auto gain is turned on.
    else if(!gain_given_ && gain_auto_given && gain_auto_)
    {
      ROS_DEBUG_STREAM("gain_auto is given and set to true (Continuous).");
    }
    // TFF
    // The provided gain is used.
    // TFT
    // Turn auto gain off, the provided gain is used.
    else if(gain_given_ && !gain_auto_given && gain_auto_)
    {
      gain_auto_ = false;
    }
    // TTF
    // Auto gain is turned off, the provided gain is used.
    else if(gain_given_ && gain_auto_given && !gain_auto_)
    {
      ROS_DEBUG_STREAM("gain_auto is given and set to false (Off)." );
    }
    // TTT
    // Turn auto gain off, the provided gain is used.
    else if(gain_given_ && gain_auto_given && gain_auto_)
    {
      gain_auto_ = false;
      
      ROS_WARN_STREAM("gain_auto is ignored because gain is given.");
    }
    
    // Get MTU size.
    if (nh.hasParam("gige/mtu_size"))
    {
      nh.getParam("gige/mtu_size", mtu_size_);
    }

    // Get inter-package delay.
    if (nh.hasParam("gige/inter_pkg_delay"))
    {
      nh.getParam("gige/inter_pkg_delay", inter_pkg_delay_);
    }

    // Get shutter mode.
    std::string shutter_param_string;
    
    nh.param<std::string>("shutter_mode", shutter_param_string, "");
    
    if (shutter_param_string == "rolling")
    {
      shutter_mode_ = SM_ROLLING;
    }
    else if (shutter_param_string == "global")
    {
      shutter_mode_ = SM_GLOBAL;
    }
    else if (shutter_param_string == "global_reset")
    {
      shutter_mode_ = SM_GLOBAL_RESET_RELEASE;
    }
    else
    {
      shutter_mode_ = SM_DEFAULT;
    }

    // Define auto flash flags.
    nh.param<bool>("auto_flash", auto_flash_, false);
    nh.param<bool>("auto_flash_line_2", auto_flash_line_2_, false);
    nh.param<bool>("auto_flash_line_3", auto_flash_line_3_, false);

    ROS_WARN("Auto flash: %i, line 2: %i , line 3: %i ", auto_flash_, auto_flash_line_2_,
              auto_flash_line_3_);
    
    validateParameterSet(nh);
    
    return;
  }

  void ArenaCameraParameter::adaptDeviceUserId(const ros::NodeHandle& nh,
                                                const std::string& device_user_id)
  {
    device_user_id_ = device_user_id;
    
    nh.setParam("device_user_id", device_user_id_);
  }

  void ArenaCameraParameter::validateParameterSet(const ros::NodeHandle& nh)
  {
    // Validate device user ID.
    if (!device_user_id_.empty())
    {
      ROS_INFO_STREAM("Trying to open camera " << device_user_id_.c_str() << ".");
    }
    else
    {
      ROS_INFO_STREAM("No device user ID set, will open the first camera device found.");
    }

    // Validate frame rate.
    if (frame_rate_ <= 0 && frame_rate_ != -1)
    {
      ROS_WARN_STREAM("Desired frame rate (" << frame_rate_ << " Hz) is invalid. It will be reset "
                      << "to the default value (10.0 Hz).");
      
      frame_rate_ = 10.0;
      
      nh.setParam("frame_rate", frame_rate_);
    }

    // Validate exposure time.
    if (exposure_given_ && (exposure_ <= 0.0 || exposure_ > 1e7))
    {
      ROS_WARN_STREAM("Desired exposure time (" << exposure_ << " us) is invalid. It will be "
                      << "to the default value (100000.0 us).");
      
      exposure_given_ = false;
    }

    // Validate gain.
    if (gain_given_ && (gain_ < 0.0 || gain_ > 1.0))
    {
      ROS_WARN_STREAM("Desired gain fraction (" << gain_ << ") is invalid. It will be reset to "
                      << "the default value (0.5).");
      
      gain_given_ = false;
    }

    if (brightness_given_ && (brightness_ < 0.0 || brightness_ > 255))
    {
      ROS_WARN_STREAM("Desired brightness (" << brightness_ << ") is invalid. It will be reset to "
                      << "the default value (70).");
      
      brightness_given_ = false;
    }

    return;
  }

  const std::string& ArenaCameraParameter::deviceUserID() const
  {
    return device_user_id_;
  }

  std::string ArenaCameraParameter::shutterModeString() const
  {
    if (shutter_mode_ == SM_ROLLING)
    {
      return "rolling";
    }
    else if (shutter_mode_ == SM_GLOBAL)
    {
      return "global";
    }
    else if (shutter_mode_ == SM_GLOBAL_RESET_RELEASE)
    {
      return "global reset";
    }
    else
    {
      return "default shutter mode";
    }
  }

  const std::string& ArenaCameraParameter::imageEncoding() const
  {
    return image_encoding_;
  }

  const std::string& ArenaCameraParameter::cameraFrame() const
  {
    return camera_frame_;
  }

  const double& ArenaCameraParameter::frameRate() const
  {
    return frame_rate_;
  }

  void ArenaCameraParameter::setFrameRate(const ros::NodeHandle& nh, const double& frame_rate)
  {
    frame_rate_ = frame_rate;
    
    nh.setParam("frame_rate", frame_rate_);
  }

  const std::string& ArenaCameraParameter::cameraInfoURL() const
  {
    return camera_info_url_;
  }

  void ArenaCameraParameter::setCameraInfoURL(const ros::NodeHandle& nh,
                                              const std::string& camera_info_url)
  {
    camera_info_url_ = camera_info_url;
    
    nh.setParam("camera_info_url", camera_info_url_);
  }
}  // namespace arena_camera