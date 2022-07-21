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
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

// ROS
#include <sensor_msgs/RegionOfInterest.h>
#include "boost/multi_array.hpp"

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <GenApiCustom.h>

// Arena node
#include <arena_camera/arena_camera_node.h>
#include <arena_camera/encoding_conversions.h>

using diagnostic_msgs::DiagnosticStatus;

namespace arena_camera
{
  // Define system, device, image, data, and node map pointers.
  Arena::ISystem* pSystem_ = nullptr;
  Arena::IDevice* pDevice_ = nullptr;
  Arena::IImage* pImage_ = nullptr;
  const uint8_t* pData_ = nullptr;
  GenApi::INodeMap* pNodeMap_ = nullptr;

  using sensor_msgs::CameraInfo;
  using sensor_msgs::CameraInfoPtr;

  ArenaCameraNode::ArenaCameraNode()
    : nh_("~")
    , arena_camera_parameter_set_()
    // Define ROS services.
    , set_binning_srv_(nh_.advertiseService("set_binning",
                                            &ArenaCameraNode::setBinningCallback, this))
    , set_roi_srv_(nh_.advertiseService("set_roi",
                                        &ArenaCameraNode::setROICallback, this))
    , set_exposure_srv_(nh_.advertiseService("set_exposure",
                                              &ArenaCameraNode::setExposureCallback, this))
    , set_gain_srv_(nh_.advertiseService("set_gain",
                                          &ArenaCameraNode::setGainCallback, this))
    , set_gamma_srv_(nh_.advertiseService("set_gamma",
                                          &ArenaCameraNode::setGammaCallback, this))
    , set_brightness_srv_(nh_.advertiseService("set_brightness",
                                                &ArenaCameraNode::setBrightnessCallback, this))
    , get_properties_srv_(nh_.advertiseService("get_properties",
                                                &ArenaCameraNode::getPropertiesCallback, this))
    , set_sleeping_srv_(nh_.advertiseService("set_sleeping",
                                              &ArenaCameraNode::setSleepingCallback, this))
    , set_user_output_srvs_()
    // Define ROS publishers.
    , it_(new image_transport::ImageTransport(nh_))
    , img_raw_pub_(it_->advertiseCamera("image_raw", 1))
    , img_rect_pub_(nullptr)
    , img_scaled_pub_(nullptr)
    , discovery_pub_(nullptr)
    , grab_imgs_raw_as_(nh_, "grab_images_raw",
                        boost::bind(&ArenaCameraNode::grabImagesRawActionExecuteCB, this, _1),
                        false)
    , grab_imgs_rect_as_(nullptr)
    , pinhole_model_(nullptr)
    , cv_bridge_img_rect_(nullptr)
    , camera_info_manager_(new camera_info_manager::CameraInfoManager(nh_))
    , is_sleeping_(false)
  {
    diagnostics_updater_.setHardwareID("none");
    diagnostics_updater_.add("camera_availability", this, &ArenaCameraNode::create_diagnostics);
    diagnostics_updater_.add("intrinsic_calibration", this,
                              &ArenaCameraNode::create_camera_info_diagnostics);
    
    diagnostics_trigger_ = nh_.createTimer(ros::Duration(2),
                                            &ArenaCameraNode::diagnostics_timer_callback_, this);

    init();
  }

  void ArenaCameraNode::create_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
  }

  void ArenaCameraNode::create_camera_info_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (camera_info_manager_->isCalibrated())
    {
      stat.summaryf(DiagnosticStatus::OK, "Intrinsic calibration found.");
    }
    else
    {
      stat.summaryf(DiagnosticStatus::ERROR, "No intrinsic calibration found.");
    }
  }

  void ArenaCameraNode::diagnostics_timer_callback_(const ros::TimerEvent&)
  {
    diagnostics_updater_.update();
  }

  void ArenaCameraNode::init()
  {
    // Read the necessary parameters for opening the desired camera from the
    // ros-parameter-server. If invalid parameter values are detected the
    // interface will reset them to the default values. The parameters may also
    // contain intrinsic calibration matrices, in case any is provided.
    arena_camera_parameter_set_.readFromRosParameterServer(nh_);

    // Set the CameraInfo URL to produce rectified image. You can substitute
    // any desired file path or comment out this line if only publishing raw
    // images.
    //  arena_camera_parameter_set_.setCameraInfoURL(nh_,
    //  "file://${ROS_HOME}/camera_info/camera.yaml");

    // Initialize the CARMA driver discovery publisher.
    discovery_pub_ = new ros::Publisher(nh_.advertise<cav_msgs::DriverStatus>("discovery", 1));
    
    discovery_msg_.name = "/hardware_interface/camera";
    discovery_msg_.camera = true;
    
    last_discovery_pub_ = ros::Time::now();

    // initAndRegister() creates the target ArenaCamera object with the
    // specified device_user_id, registers the Software-Trigger-Mode, starts
    // communication with the device and enables the desired startup settings.
    if (!initAndRegister())
    {
      discovery_msg_.status = cav_msgs::DriverStatus::OFF;
      
      if (last_discovery_pub_ == ros::Time(0) ||
          (ros::Time::now() - last_discovery_pub_).toSec() > 0.8)
      {
          discovery_pub_->publish(discovery_msg_);
          last_discovery_pub_ = ros::Time::now();
      }

      ros::shutdown();
      
      return;
    }

    // startGrabbing() starts the grabbing process using the desired image
    // settings.
    if (!startGrabbing())
    {
      discovery_msg_.status = cav_msgs::DriverStatus::OFF;
      
      if (last_discovery_pub_ == ros::Time(0) ||
          (ros::Time::now() - last_discovery_pub_).toSec() > 0.8)
      {
          discovery_pub_->publish(discovery_msg_);
          last_discovery_pub_ = ros::Time::now();
      }
      
      ros::shutdown();
      
      return;
    }
  }

  bool createDevice(const std::string& device_user_id_to_open)
  {
    // Create a system object and get a list of connected devices.
    pSystem_ = Arena::OpenSystem();
    pSystem_->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();

    // If no device is connected, shut down. Otherwise, open the first
    // connected device, unless the user specifies a particular device.
    if (deviceInfos.size() == 0)
    {
      Arena::CloseSystem(pSystem_);
      pSystem_ = nullptr;
      
      return false;
    }
    else
    {
      if (device_user_id_to_open.empty())
      {
        pDevice_ = pSystem_->CreateDevice(deviceInfos[0]);
        
        return true;
      }
      else
      {
        std::vector<Arena::DeviceInfo>::iterator it;
        bool found_desired_device = false;

        for (it = deviceInfos.begin(); it != deviceInfos.end(); ++it)
        {
          std::string device_user_id_found(it->UserDefinedName());
          
          if ((0 == device_user_id_to_open.compare(device_user_id_found)) ||
              (device_user_id_to_open.length() < device_user_id_found.length() &&
              (0 ==
                device_user_id_found.compare(device_user_id_found.length() - device_user_id_to_open.length(),
                                              device_user_id_to_open.length(), device_user_id_to_open))))
          {
            found_desired_device = true;
            
            break;
          }
        }
        if (found_desired_device)
        {
          ROS_INFO_STREAM("Found the desired camera with DeviceUserID "
                          << device_user_id_to_open << ".");

          pDevice_ = pSystem_->CreateDevice(*it);
          
          return true;
        }
        else
        {
          ROS_ERROR_STREAM("Couldn't find the camera that matches the "
                          << "given DeviceUserID: " << device_user_id_to_open << "! "
                          << "Either the ID is wrong or the camera is not yet connected.");
          
          return false;
        }
      }
    }
  }

  bool ArenaCameraNode::initAndRegister()
  {
    // Initialize the camera.
    bool device_found_ = false;
    device_found_ = createDevice(arena_camera_parameter_set_.deviceUserID());

    if (device_found_ == false)
    {
      // Wait and retry until a camera is present.
      ros::Time end = ros::Time::now() + ros::Duration(15.0);
      ros::Rate r(0.5);
      
      while (ros::ok() && device_found_ == false)
      {
        device_found_ = createDevice(arena_camera_parameter_set_.deviceUserID());
        
        if (ros::Time::now() > end)
        {
          ROS_WARN_STREAM("No camera present. Keep waiting...");
          end = ros::Time::now() + ros::Duration(15.0);
        }
        
        discovery_msg_.status = cav_msgs::DriverStatus::OFF;
        
        if (last_discovery_pub_ == ros::Time(0) ||
            (ros::Time::now() - last_discovery_pub_).toSec() > 0.8)
        {
            discovery_pub_->publish(discovery_msg_);
            last_discovery_pub_ = ros::Time::now();
        }
        
        r.sleep();
        ros::spinOnce();
      }
    }
    else
    {
      ROS_INFO_STREAM("Camera " << arena_camera_parameter_set_.deviceUserID() << " is found!");

      // Enable auto packet size negotiation. If it cannot be enabled, set the
      // packet size manually to the maximum possible value.
      Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                                "StreamAutoNegotiatePacketSize", true);
      
      GenApi::CIntegerPtr pDeviceStreamChannelPacketSize = pDevice_->GetNodeMap()->GetNode("DeviceStreamChannelPacketSize");
      
      if (!pDeviceStreamChannelPacketSize ||
          !GenApi::IsReadable(pDeviceStreamChannelPacketSize) ||
          !GenApi::IsWritable(pDeviceStreamChannelPacketSize))
      {
        throw GenICam::GenericException("DeviceStreamChannelPacketSize node not found/readable/writable.",
                                        __FILE__, __LINE__);
      }

      if (!Arena::GetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                                      "StreamAutoNegotiatePacketSize"))
      {
        ROS_INFO_STREAM("Maximum device stream channel packet size is "
                      << pDeviceStreamChannelPacketSize->GetMax()
                      << " "
                      << pDeviceStreamChannelPacketSize->GetUnit()
                      << ".");
        ROS_INFO_STREAM("Manually setting packet size to the maximum possible value.");
        pDeviceStreamChannelPacketSize->SetValue(pDeviceStreamChannelPacketSize->GetMax());
      }
      else
      {
        ROS_INFO_STREAM("Auto negotiation of packet size enabled.");
      }
    }

    if (!ros::ok())
    {
      return false;
    }

    return true;
  }

  // Get the ROI information (height, width, offset).
  sensor_msgs::RegionOfInterest currentROI()
  {
    pImage_ = pDevice_->GetImage(5000);
    
    sensor_msgs::RegionOfInterest roi;
    
    roi.width = pImage_->GetWidth();
    roi.height = pImage_->GetHeight();
    
    roi.x_offset = pImage_->GetOffsetX();
    roi.y_offset = pImage_->GetOffsetY();
    
    return roi;
  }

  // Get the gamma value.
  float currentGamma()
  {
    GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");

    if (!pGamma || !GenApi::IsReadable(pGamma))
    {
      ROS_WARN_STREAM("No gamma value, returning -1.");
      
      return -1.0;
    }
    else
    {
      float gammaValue = pGamma->GetValue();
      
      return gammaValue;
    }
  }

  // Get the horizontal binning value.
  int64_t currentBinningX()
  {
    GenApi::CIntegerPtr BinningHorizontal = pDevice_->GetNodeMap()->GetNode("BinningHorizontal");

    if (!BinningHorizontal || !GenApi::IsReadable(BinningHorizontal))
    {
      ROS_WARN_STREAM("No horizontal binning value, returning -1.");
      
      return -1.0;
    }
    else
    {
      float binningXValue = BinningHorizontal->GetValue();
      
      return binningXValue;
    }
  }

  // Get the vertical binning value.
  int64_t currentBinningY()
  {
    GenApi::CIntegerPtr BinningVertical = pDevice_->GetNodeMap()->GetNode("BinningVertical");

    if (!BinningVertical || !GenApi::IsReadable(BinningVertical))
    {
      ROS_WARN_STREAM("No vertical binning value, returning -1.");
      
      return -1.0;
    }
    else
    {
      float binningYValue = BinningVertical->GetValue();
      
      return binningYValue;
    }
  }

  // Get the current digital gain value.
  float currentGain()
  {
    GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");

    if (!pGain || !GenApi::IsReadable(pGain))
    {
      ROS_WARN_STREAM("No gain value.");
      
      return -1.0;
    }
    else
    {
      float gainValue = pGain->GetValue();
      
      return gainValue;
    }
  }

  // Get the current exposure time.
  float currentExposure()
  {
    GenApi::CFloatPtr pExposureTime = pDevice_->GetNodeMap()->GetNode("ExposureTime");

    if (!pExposureTime || !GenApi::IsReadable(pExposureTime))
    {
      ROS_WARN_STREAM("No exposure time value, returning -1.");
      
      return -1.0;
    }
    else
    {
      float exposureValue = pExposureTime->GetValue();
      
      return exposureValue;
    }
  }

  // Get the current ROS image encoding.
  std::string currentROSEncoding()
  {
    std::string gen_api_encoding(Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                                        "PixelFormat"));
    std::string ros_encoding("");
    
    if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding))
    {
      std::stringstream ss;
      ss << "No ROS equivalent to GenAPI encoding '"
          << gen_api_encoding
          << "' found! This is bad because this case should never occur!";
      throw std::runtime_error(ss.str());
      
      return "NO_ENCODING";
    }
    
    return ros_encoding;
  }

  // Set the GenAPI image encoding based on the provided ROS encoding.
  bool ArenaCameraNode::setImageEncoding(const std::string& ros_encoding)
  {
    std::string gen_api_encoding;
    bool conversion_found = encoding_conversions::ros2GenAPI(ros_encoding, gen_api_encoding);
    
    if (!conversion_found)
    {
      if (ros_encoding.empty())
      {
        return false;
      }
      else
      {
        std::string fallbackPixelFormat = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                                                  "PixelFormat").c_str();
        ROS_ERROR_STREAM("Can't convert ROS encoding '" << ros_encoding
                                                        << "' to a corresponding GenAPI encoding!"
                                                        << " Will use current pixel format ("
                                                        << fallbackPixelFormat
                                                        << ") as fallback!"); 
        
        return false;
      }
    }
    try
    {
      GenApi::CEnumerationPtr pPixelFormat = pDevice_->GetNodeMap()->GetNode("PixelFormat");
      
      if (GenApi::IsWritable(pPixelFormat))
      {
        Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                "PixelFormat",
                                                gen_api_encoding.c_str());
        
        if (currentROSEncoding() == "16UC3" || currentROSEncoding() == "16UC4")
          ROS_WARN_STREAM("ROS grabbing image data from 3D pixel format, unable to display in "
                          << "image viewer.");
      }
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while setting target image encoding to '"
                        << ros_encoding << "': " << e.GetDescription());
      
      return false;
    }

    return true;
  }

  bool ArenaCameraNode::startGrabbing()
  {
    auto pNodeMap = pDevice_->GetNodeMap();

    try
    {
      // Set image encoding.
      setImageEncoding(arena_camera_parameter_set_.imageEncoding());

      // Enable software trigger.
      GenApi::CStringPtr pTriggerMode = pNodeMap->GetNode("TriggerMode");
      
      if (GenApi::IsWritable(pTriggerMode))
      {
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode", "On");
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource", "Software");
      }

      // Get the desired, current, and maximum possible frame rate values.
      auto cmdlnParamFrameRate = arena_camera_parameter_set_.frameRate();
      auto currentFrameRate = Arena::GetNodeValue<double>(pNodeMap , "AcquisitionFrameRate");
      auto maximumFrameRate = GenApi::CFloatPtr(pNodeMap->GetNode("AcquisitionFrameRate"))->GetMax();

      // If the desired frame rate is larger than the maximum possible frame
      // rate, set frame rate to the maximum value.
      if (cmdlnParamFrameRate >= trunc(1000 * maximumFrameRate) / 1000.0)
      {
        arena_camera_parameter_set_.setFrameRate(nh_, trunc(1000 * maximumFrameRate) / 1000.0);
        Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
        Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate",
                                    trunc(1000 * maximumFrameRate) / 1000.0);
        
        ROS_WARN("Desired frame rate (%.2f Hz) is higher than the maximum value possible. Device "
                  "frame rate will be limited to the maximum possible value (%.2f Hz).",
                  cmdlnParamFrameRate, maximumFrameRate);
      }
      // Device frame rate is set to its maximum possible value if the desired
      // frame rate is set to -1.
      else if (cmdlnParamFrameRate == -1)
      {
        arena_camera_parameter_set_.setFrameRate(nh_, trunc(1000 * maximumFrameRate) / 1000.0);
        Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
        Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate",
                                    trunc(1000 * maximumFrameRate) / 1000.0);
        
        ROS_WARN("Device frame rate set to the maximum possible value (%.2f Hz).",
                  maximumFrameRate);
      }
      // Desired frame rate is valid.
      else
      {
        Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
        Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate", cmdlnParamFrameRate);
        
        ROS_INFO("Device frame rate set to %.2f Hz.", cmdlnParamFrameRate);
      }

      // Set auto exposure to either continuous or off. The exposure_auto_
      // parameter is automatically set to false by
      // readFromRosParameterServer() if exposure_given_ is true.
      if (arena_camera_parameter_set_.exposure_auto_)
      {
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Continuous");
        
        ROS_INFO_STREAM("Setting auto exposure to continuous.");
      }
      else
      {
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");
        
        ROS_INFO_STREAM("Setting auto exposure to off.");
      }

      if (arena_camera_parameter_set_.exposure_given_)
      {
        float reached_exposure;
        
        if (setExposure(arena_camera_parameter_set_.exposure_, reached_exposure))
        {
          ROS_INFO_STREAM("Setting exposure to " << arena_camera_parameter_set_.exposure_
                          << ", reached: " << reached_exposure << ".");
        }
      }

      // Set auto gain to either continuous or off. The gain_auto_ parameter
      // is automatically set to false by readFromRosParameterServer() if
      // gain_given_ is true.
      if (arena_camera_parameter_set_.gain_auto_)
      {
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Continuous");
        
        ROS_INFO_STREAM("Setting auto gain to continuous.");
      }
      else
      {
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
        
        ROS_INFO_STREAM("Setting auto gain to off.");
      }

      if (arena_camera_parameter_set_.gain_given_)
      {
        float reached_gain;
        
        if (setGain(arena_camera_parameter_set_.gain_, reached_gain))
        {
          ROS_INFO_STREAM("Setting gain to: " << arena_camera_parameter_set_.gain_
                          << ", reached: " << reached_gain << ".");
        }
      }

      // Set gamma (if supported by the camera).
      if (arena_camera_parameter_set_.gamma_given_)
      {
        float reached_gamma;
        
        if (setGamma(arena_camera_parameter_set_.gamma_, reached_gamma))
        {
          ROS_INFO_STREAM("Setting gamma to " << arena_camera_parameter_set_.gamma_
                          << ", reached: " << reached_gamma << ".");
        }
      }

      // Initialize the CameraInfo message, assuming no calibration file is
      // provided.
      CameraInfo initial_cam_info;
      setupInitialCameraInfo(initial_cam_info);
      camera_info_manager_->setCameraInfo(initial_cam_info);

      if (arena_camera_parameter_set_.cameraInfoURL().empty() ||
          !camera_info_manager_->validateURL(arena_camera_parameter_set_.cameraInfoURL()))
      {
        ROS_INFO_STREAM("CameraInfoURL needed for image rectification! ROS parameter: '"
                        << nh_.getNamespace() << "/camera_info_url' = '"
                        << arena_camera_parameter_set_.cameraInfoURL() << "' is invalid!");
        ROS_DEBUG_STREAM("CameraInfoURL should have the following style: "
                          << "'file:///full/path/to/local/file.yaml' or "
                          << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
        ROS_WARN_STREAM("Will only publish distorted images to the /image_raw topic!");
      }
      else
      {
        // Override the initial CameraInfo if the provided URL is valid.
        if (camera_info_manager_->loadCameraInfo(arena_camera_parameter_set_.cameraInfoURL()))
        {
          setupRectification();
          
          // Set the correct TF frame_id.
          CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
          cam_info->header.frame_id = img_raw_msg_.header.frame_id;
          camera_info_manager_->setCameraInfo(*cam_info);
        }
        else
        {
          ROS_WARN_STREAM("Will only publish distorted images to the /image_raw topic!");
        }
      }

      // Switch binning mode.
      if (arena_camera_parameter_set_.binning_x_given_ || arena_camera_parameter_set_.binning_y_given_)
      {
        GenApi::CEnumerationPtr pBinningSelector = pDevice_->GetNodeMap()->GetNode("BinningSelector");

        if (GenApi::IsWritable(pBinningSelector))
        {
          if (arena_camera_parameter_set_.sensor_binning_)
          {
            Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                    "BinningSelector",
                                                    "Sensor");

            ROS_INFO_STREAM("Using sensor binning.");
          }
          else
          {
            Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                    "BinningSelector",
                                                    "Digital");

            ROS_INFO_STREAM("Using digital binning.");
          }
        }
        else
        {
          ROS_WARN_STREAM("Cannot change binning mode.");
        }
      }

      // Apply horizontal binning.
      if (arena_camera_parameter_set_.binning_x_given_)
      {
        size_t reached_binning_x;

        if (setBinningX(arena_camera_parameter_set_.binning_x_, reached_binning_x))
        {
          ROS_INFO_STREAM("Setting horizontal binning to " << reached_binning_x << ".");
          ROS_WARN_STREAM("Image width in the CameraInfo message will be adjusted to keep the "
                          << "horizontal binning value in that message at 1.");
        }
      }

      // Apply vertical binning.
      if (arena_camera_parameter_set_.binning_y_given_)
      {
        size_t reached_binning_y;
        
        if (setBinningY(arena_camera_parameter_set_.binning_y_, reached_binning_y))
        {
          ROS_INFO_STREAM("Setting vertical binning to " << reached_binning_y << ".");
          ROS_WARN_STREAM("Image height in the CameraInfo message will be adjusted to keep the "
                          << "vertical binning value in that message at 1.");
        }
      }

      Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetTLStreamNodeMap(),
                                              "StreamBufferHandlingMode",
                                              "NewestOnly");

      // Start camera stream and arm the software trigger.
      pDevice_->StartStream();
      bool isTriggerArmed = false;

      if (GenApi::IsWritable(pTriggerMode))
      {
        do
        {
          isTriggerArmed = Arena::GetNodeValue<bool>(pNodeMap, "TriggerArmed");
        } while (isTriggerArmed == false);
        
        Arena::ExecuteNode(pNodeMap, "TriggerSoftware");
      }

      // Grab the first image, along with frame ID, encoding, height and width
      // information.
      pImage_ = pDevice_->GetImage(5000);
      pData_ = pImage_->GetData();

      img_raw_msg_.header.frame_id = arena_camera_parameter_set_.cameraFrame();
      img_raw_msg_.encoding = currentROSEncoding();
      img_raw_msg_.height = pImage_->GetHeight();
      img_raw_msg_.width = pImage_->GetWidth();

      // Image step is a full row length in bytes, i.e. image size is
      // (step * rows). Image pixel depth already contains information about
      // the number of channels.
      bitDepth = pImage_->GetBitsPerPixel();
      img_raw_msg_.step = img_raw_msg_.width * (bitDepth / 8);

      img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
      memcpy(&img_raw_msg_.data[0], pImage_->GetData(), img_raw_msg_.height * img_raw_msg_.step);

      // Set up a publisher for downsampled images when the raw image has 16
      // or 24 bits. It will also publish a debayered image when the raw image
      // has 8-bit Bayer pattern encoding. It will just publish the raw image
      // if it has 8-bit three channel (RGB or BGR) encoding.
      img_scaled_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::Image>("image_scaled", 1));

      // Prepare to set brightness.
      brightness_given = false;
      
      if (arena_camera_parameter_set_.brightness_given_ &&
            (arena_camera_parameter_set_.exposure_auto_ ||
              arena_camera_parameter_set_.gain_auto_))
      {
        brightness_given = true;
        brightness_set = false;

        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
      }
    }
    catch (GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("Error occurred while grabbing the first image: \r\n" << e.GetDescription());
      
      return false;
    }

    if (!camera_info_manager_->setCameraName(
            std::string(Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID").c_str())))
    {
      // A valid name only contains alphanumeric values and '_'.
      ROS_WARN_STREAM(
          "[" << std::string(Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID").c_str())
              << "] is not a valid name for camera_info_manager.");
    }

    grab_imgs_raw_as_.start();

    ROS_INFO_STREAM("Startup settings: "
                    << "encoding = '" << currentROSEncoding() << "', "
                    << "binning = [" << currentBinningX() << ", " << currentBinningY() << "], "
                    << "exposure = " << currentExposure() << " us, "
                    << "gain = " << currentGain() << " dB, "
                    << "gamma = " << currentGamma() << ", "
                    << "shutter mode = " << arena_camera_parameter_set_.shutterModeString()
                    << ".");

    pDevice_->RequeueBuffer(pImage_);

    return true;
  }

  // Set up image rectification if CameraInfo is provided.
  void ArenaCameraNode::setupRectification()
  {
    if (!img_rect_pub_)
    {
      img_rect_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::Image>("image_rect", 1));
    }

    if (!grab_imgs_rect_as_)
    {
      grab_imgs_rect_as_ = new GrabImagesAS(nh_,
                                            "grab_images_rect",
                                            boost::bind(&ArenaCameraNode::grabImagesRectActionExecuteCB, this, _1),
                                            false);
      grab_imgs_rect_as_->start();
    }

    if (!pinhole_model_)
    {
      pinhole_model_ = new image_geometry::PinholeCameraModel();
    }

    pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
    
    if (!cv_bridge_img_rect_)
    {
      cv_bridge_img_rect_ = new cv_bridge::CvImage();
    }
    cv_bridge_img_rect_->header = img_raw_msg_.header;
    cv_bridge_img_rect_->encoding = img_raw_msg_.encoding;
  }

  struct CameraPublisherImpl
  {
    image_transport::Publisher image_pub_;
    ros::Publisher info_pub_;
    bool unadvertised_;
  };

  class CameraPublisherLocal
  {
    public:
      struct Impl;
      typedef boost::shared_ptr<Impl> ImplPtr;
      typedef boost::weak_ptr<Impl> ImplWPtr;

      CameraPublisherImpl* impl_;
  };

  uint32_t ArenaCameraNode::getNumSubscribersRaw() const
  {
    return ((CameraPublisherLocal*)(&img_raw_pub_))->impl_->image_pub_.getNumSubscribers();
  }

  void ArenaCameraNode::spin()
  {
    if (camera_info_manager_->isCalibrated())
    {
      ROS_INFO_ONCE("Camera is calibrated.");
    }
    else
    {
      ROS_INFO_ONCE("Camera is not calibrated.");
    }

    // If the device cannot be detected, destroy device and system pointers
    // and try to reinitialize the device.
    if (pDevice_->IsConnected() == false)
    {
      ROS_ERROR("Camera has been removed, trying to reset...");
      
      pSystem_->DestroyDevice(pDevice_);
      pDevice_ = nullptr;

      Arena::CloseSystem(pSystem_);
      pSystem_ = nullptr;

      for (ros::ServiceServer& user_output_srv : set_user_output_srvs_)
      {
        user_output_srv.shutdown();
      }

      discovery_msg_.status = cav_msgs::DriverStatus::FAULT;
      
      if (last_discovery_pub_ == ros::Time(0) ||
          (ros::Time::now() - last_discovery_pub_).toSec() > 0.8)
      {
          discovery_pub_->publish(discovery_msg_);
          last_discovery_pub_ = ros::Time::now();
      }

      // Sleep for 0.5 seconds.
      ros::Duration(0.5).sleep();
      init();

      return;
    }

    if (!isSleeping() && (img_raw_pub_.getNumSubscribers() ||
                          img_scaled_pub_->getNumSubscribers() ||
                          getNumSubscribersRect()))
    {
      if (getNumSubscribersRaw() || getNumSubscribersRect() ||
          img_scaled_pub_->getNumSubscribers())
      {
        if (!grabImage())
        {
          ROS_INFO("Did not get image.");

          discovery_msg_.status = cav_msgs::DriverStatus::FAULT;
          
          if (last_discovery_pub_ == ros::Time(0) ||
              (ros::Time::now() - last_discovery_pub_).toSec() > 0.8)
          {
              discovery_pub_->publish(discovery_msg_);
              last_discovery_pub_ = ros::Time::now();
          }

          return;
        }
      }

      // Publish raw image data if there is a subscriber.
      if (img_raw_pub_.getNumSubscribers() > 0)
      {
        // Get the updated CameraInfo, because it may have been changed by a
        // service call.
        sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->header.stamp = img_raw_msg_.header.stamp;

        // Publish via image_transport.
        img_raw_pub_.publish(img_raw_msg_, *cam_info);

        ROS_INFO_ONCE("Publishing raw images.");
        
        discovery_msg_.status = cav_msgs::DriverStatus::OPERATIONAL;
      }

      // Publish rectified (calibrated) images if camera calibration
      // information is provided and there is a subscriber for rectified
      // images.
      if (getNumSubscribersRect() > 0 && camera_info_manager_->isCalibrated())
      {
        assert(pinhole_model_->initialized());
        
        cv_bridge_img_rect_->header.stamp = img_raw_msg_.header.stamp;
        
        cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(img_raw_msg_,
                                                                img_raw_msg_.encoding);
        
        pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
        pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect_->image);
        
        img_rect_pub_->publish(*cv_bridge_img_rect_);
        
        ROS_INFO_ONCE("Publishing rectified images.");
        
        discovery_msg_.status = cav_msgs::DriverStatus::OPERATIONAL;
      }

      // Because RViz cannot display images with a bit depth higher than 8,
      // the raw 16- or 24-bit image is downsampled to an 8-bit image and
      // published on a different topic. If image encoding is a Bayer pattern
      // the image will be debayered, though this will add about 20 ms of
      // delay. 12-bit packed image encodings are not supported.
      if (img_scaled_pub_->getNumSubscribers() > 0)
      {
        img_scaled_msg_.header = img_raw_msg_.header;
        img_scaled_msg_.height = img_raw_msg_.height;
        img_scaled_msg_.width = img_raw_msg_.width;

        if (sensor_msgs::image_encodings::isBayer(img_raw_msg_.encoding))
        {
          // We assume that the sensor has a Bayer RGGB pattern, this can be
          // changed if the sensor has a different pattern.
          img_scaled_msg_.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;

          size_t scalingFactor = bitDepth / 8;

          img_scaled_msg_.step = img_raw_msg_.step / scalingFactor;
          img_scaled_msg_.data.resize(img_scaled_msg_.height * img_scaled_msg_.step);

          // Since raw image data is little-endian, keep the last byte for each
          //pixel and discard the rest.
          for (std::size_t i = 0; i < img_scaled_msg_.height * img_scaled_msg_.step; ++i)
          {
            img_scaled_msg_.data[i] = img_raw_msg_.data[scalingFactor * (i + 1) - 1];
          }

          // Use OpenCV to debayer the resulting image.
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_scaled_msg_,
                                                              img_scaled_msg_.encoding);
          cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(img_scaled_msg_,
                                                              img_scaled_msg_.encoding);
          
          cv::cvtColor(cv_ptr->image, cv_rgb->image, cv::COLOR_BayerBG2RGB);

          cv_rgb->encoding = sensor_msgs::image_encodings::RGB8;

          img_scaled_pub_->publish(cv_rgb->toImageMsg());
        }
        else
        {
          if (sensor_msgs::image_encodings::isRGB(img_raw_msg_.encoding))
          {
            img_scaled_msg_.encoding = sensor_msgs::image_encodings::RGB8;
          }
          else if (sensor_msgs::image_encodings::isBGR(img_raw_msg_.encoding))
          {
            img_scaled_msg_.encoding = sensor_msgs::image_encodings::BGR8;
          }
          else
          {
            ROS_WARN("Image encoding is not supported.");

            return;
          }

          size_t scalingFactor = bitDepth / 24;

          img_scaled_msg_.step = img_raw_msg_.step / scalingFactor;
          img_scaled_msg_.data.resize(img_scaled_msg_.height * img_scaled_msg_.step);

          // Since raw image data is little-endian, keep the last byte for each
          //pixel and discard the rest.
          for (std::size_t i = 0; i < img_scaled_msg_.height * img_scaled_msg_.step; ++i)
          {
            img_scaled_msg_.data[i] = img_raw_msg_.data[scalingFactor * (i + 1) - 1];
          }

          img_scaled_pub_->publish(img_scaled_msg_);
        }

        ROS_INFO_ONCE("Publishing scaled images.");

        discovery_msg_.status = cav_msgs::DriverStatus::OPERATIONAL;
      }

      // Set brightness.
      if (brightness_given && !brightness_set)
      {
        // Set brightness, only if either auto exposure or auto gain is turned
        // on.
        if (arena_camera_parameter_set_.brightness_given_ &&
            (arena_camera_parameter_set_.exposure_auto_ ||
              arena_camera_parameter_set_.gain_auto_))
        {
          int reached_brightness;

          setBrightness(arena_camera_parameter_set_.brightness_, reached_brightness,
                        arena_camera_parameter_set_.exposure_auto_,
                        arena_camera_parameter_set_.gain_auto_);
        }

        brightness_set = true;
      }

      if (last_discovery_pub_ == ros::Time(0) ||
          (ros::Time::now() - last_discovery_pub_).toSec() > 0.8)
      {
          discovery_pub_->publish(discovery_msg_);
          last_discovery_pub_ = ros::Time::now();
      }
    }
  }

  bool ArenaCameraNode::grabImage()
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    
    try
    {
      GenApi::CStringPtr pTriggerMode = pDevice_->GetNodeMap()->GetNode("TriggerMode");
      
      if (GenApi::IsWritable(pTriggerMode))
      {
        bool isTriggerArmed = false;

        do
        {
          isTriggerArmed = Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
        } while (isTriggerArmed == false);

        Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
      }

      pImage_ = pDevice_->GetImage(5000);
      pData_ = pImage_->GetData();

      img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
      memcpy(&img_raw_msg_.data[0], pImage_->GetData(), img_raw_msg_.height * img_raw_msg_.step);

      img_raw_msg_.header.stamp = ros::Time::now();

      pDevice_->RequeueBuffer(pImage_);
      
      return true;
    }
    catch (GenICam::GenericException& e)
    {
      return false;
    }
  }

  void ArenaCameraNode::grabImagesRawActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal)
  {
    camera_control_msgs::GrabImagesResult result;
    
    result = grabImagesRaw(goal, &grab_imgs_raw_as_);
    grab_imgs_raw_as_.setSucceeded(result);
  }

  void ArenaCameraNode::grabImagesRectActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal)
  {
    camera_control_msgs::GrabImagesResult result;
    
    if (!camera_info_manager_->isCalibrated())
    {
      result.success = false;
      grab_imgs_rect_as_->setSucceeded(result);
      
      return;
    }
    else
    {
      result = grabImagesRaw(goal, std::ref(grab_imgs_rect_as_));
      
      if (!result.success)
      {
        grab_imgs_rect_as_->setSucceeded(result);
        
        return;
      }

      for (std::size_t i = 0; i < result.images.size(); ++i)
      {
        cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(result.images[i],
                                                                result.images[i].encoding);
        cv_bridge::CvImage cv_bridge_img_rect;

        cv_bridge_img_rect.header = result.images[i].header;
        cv_bridge_img_rect.encoding = result.images[i].encoding;
        
        pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
        pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect.image);
        
        cv_bridge_img_rect.toImageMsg(result.images[i]);
      }

      grab_imgs_rect_as_->setSucceeded(result);
    }
  }

  camera_control_msgs::GrabImagesResult ArenaCameraNode::grabImagesRaw(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal,
                                                                        GrabImagesAS* action_server)
  {
    camera_control_msgs::GrabImagesResult result;
    camera_control_msgs::GrabImagesFeedback feedback;

  #if DEBUG
    std::cout << *goal << std::endl;
  #endif

    // Verify the provided information about exposure times, gain, gamma, and
    // brightness.
    if (goal->exposure_given && goal->exposure_times.empty())
    {
      ROS_ERROR_STREAM("GrabImagesRaw action server received a request and 'exposure_given' is "
                        << "set to true, but the 'exposure_times' vector is empty! Not enough "
                        << "information has been given to execute acquisition!");
      
      result.success = false;
      
      return result;
    }

    if (goal->gain_given && goal->gain_values.empty())
    {
      ROS_ERROR_STREAM("GrabImagesRaw action server received a request and 'gain_given' is set to "
                        << "true, but the 'gain_values' vector is empty! Not enough information "
                        << "has been given to execute acquisition!");
      
      result.success = false;
      
      return result;
    }

    if (goal->brightness_given && goal->brightness_values.empty())
    {
      ROS_ERROR_STREAM("GrabImagesRaw action server received a request and 'brightness_given' is "
                        << "set to true, but the 'brightness_values' vector is empty! Not enough "
                        << "information has been given to execute acquisition!");
      
      result.success = false;
      
      return result;
    }

    if (goal->gamma_given && goal->gamma_values.empty())
    {
      ROS_ERROR_STREAM("GrabImagesRaw action server received a request and 'gamma_given' is set "
                        << "to true, but the 'gamma_values' vector is empty! Not enough "
                        << "information has been given to execute acquisition!");
      
      result.success = false;
      
      return result;
    }

    std::vector<size_t> candidates;

    candidates.resize(4);  // gain, exposure, gamma, brightness
    candidates.at(0) = goal->gain_given ? goal->gain_values.size() : 0;
    candidates.at(1) = goal->exposure_given ? goal->exposure_times.size() : 0;
    candidates.at(2) = goal->brightness_given ? goal->brightness_values.size() : 0;
    candidates.at(3) = goal->gamma_given ? goal->gamma_values.size() : 0;

    size_t n_images = *std::max_element(candidates.begin(), candidates.end());

    if (goal->exposure_given && goal->exposure_times.size() != n_images)
    {
      ROS_ERROR_STREAM("The size of requested exposure times does not match the size of the "
                        << "requested values for brightness, gain or gamma! Cannot grab image!");

      result.success = false;
      
      return result;
    }

    if (goal->gain_given && goal->gain_values.size() != n_images)
    {
      ROS_ERROR_STREAM("The size of requested gain values does not match the size of the "
                        << "requested values for exposure time, brightness, or gamma! Cannot grab "
                        << "image!");
      
      result.success = false;
      
      return result;
    }

    if (goal->gamma_given && goal->gamma_values.size() != n_images)
    {
      ROS_ERROR_STREAM("The size of requested gamma values does not match the size of the "
                        << "requested values for exposure time, brightness, or gain! Cannot grab "
                        << "image!");
      
      result.success = false;
      
      return result;
    }

    if (goal->brightness_given && goal->brightness_values.size() != n_images)
    {
      ROS_ERROR_STREAM("The size of requested brightness values does not match the size of the "
                        << "requested values for exposure time, gamma, or gain! Cannot grab "
                        << "image!");

      result.success = false;
      
      return result;
    }

    if (goal->brightness_given && !(goal->exposure_auto || goal->gain_auto))
    {
      ROS_ERROR_STREAM("Error while executing GrabImagesRawAction: a target brightness is "
                        << "provided but exposure time AND gain are manually set, so it is "
                        << "impossible to reach the desired brightness.");
      
      result.success = false;
      
      return result;
    }

    result.images.resize(n_images);
    result.reached_exposure_times.resize(n_images);
    result.reached_gain_values.resize(n_images);
    result.reached_gamma_values.resize(n_images);
    result.reached_brightness_values.resize(n_images);

    result.success = true;

    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

    float previous_exp, previous_gain, previous_gamma;
    
    if (goal->exposure_given)
    {
      previous_exp = currentExposure();
    }
    if (goal->gain_given)
    {
      previous_gain = currentGain();
    }
    if (goal->gamma_given)
    {
      previous_gamma = currentGamma();
    }
    if (goal->brightness_given)
    {
      previous_exp = currentExposure();
      previous_gain = currentGain();
    }

    for (std::size_t i = 0; i < n_images; ++i)
    {
      if (goal->exposure_given)
      {
        result.success = setExposure(goal->exposure_times[i], result.reached_exposure_times[i]);
      }
      if (goal->gain_given)
      {
        result.success = setGain(goal->gain_values[i], result.reached_gain_values[i]);
      }
      if (goal->gamma_given)
      {
        result.success = setGamma(goal->gamma_values[i], result.reached_gamma_values[i]);
      }
      if (goal->brightness_given)
      {
        int reached_brightness;
        
        result.success = setBrightness(goal->brightness_values[i], reached_brightness,
                                        goal->exposure_auto, goal->gain_auto);
        result.reached_brightness_values[i] = static_cast<float>(reached_brightness);
        /*
        result.reached_exposure_times[i] = currentExposure();
        result.reached_gain_values[i] = currentGain();
        */
      }
      if (!result.success)
      {
        ROS_ERROR_STREAM("Error while setting one of the desired image properties in "
                        << "GrabImagesRawActionCB. Aborting!");
        
        break;
      }

      sensor_msgs::Image& img = result.images[i];
      
      img.encoding = currentROSEncoding();
      img.height = pImage_->GetHeight();
      img.width = pImage_->GetWidth();
      
      // Image step is a full row length in bytes, i.e. image size is
      // (step * rows). Image pixel depth already contains information about
      // the number of channels.
      img.step = img.width * (pImage_->GetBitsPerPixel() / 8);

      img.header.stamp = ros::Time::now();
      img.header.frame_id = cameraFrame();

      feedback.curr_nr_images_taken = i + 1;

      if (action_server != nullptr)
      {
        action_server->publishFeedback(feedback);
      }
    }
    if (camera_info_manager_)
    {
      sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
      
      result.cam_info = *cam_info;
    }

    // Restore previous settings.
    float reached_val;

    if (goal->exposure_given)
    {
      setExposure(previous_exp, reached_val);
    }
    if (goal->gain_given)
    {
      setGain(previous_gain, reached_val);
    }
    if (goal->gamma_given)
    {
      setGamma(previous_gamma, reached_val);
    }
    if (goal->brightness_given)
    {
      setGain(previous_gain, reached_val);
      setExposure(previous_exp, reached_val);
    }
    
    return result;
  }

  bool ArenaCameraNode::setUserOutputCB(const int output_id,
                                        camera_control_msgs::SetBool::Request& req,
                                        camera_control_msgs::SetBool::Response& res)
  {
    /*
    res.success = arena_camera_->setUserOutput(output_id, req.data);
    */
    
    return true;
  }

  bool ArenaCameraNode::setAutoflash(const int output_id,
                                      camera_control_msgs::SetBool::Request& req,
                                      camera_control_msgs::SetBool::Response& res)
  {
    ROS_INFO("AutoFlashCB: %i -> %i", output_id, req.data);
    
    std::map<int, bool> auto_flashes;
    
    auto_flashes[output_id] = req.data;
    /*
    arena_camera_->setAutoflash(auto_flashes);
    */
    res.success = true;
    
    return true;
  }

  const double& ArenaCameraNode::frameRate() const
  {
    return arena_camera_parameter_set_.frameRate();
  }

  const std::string& ArenaCameraNode::cameraFrame() const
  {
    return arena_camera_parameter_set_.cameraFrame();
  }

  uint32_t ArenaCameraNode::getNumSubscribersRect() const
  {
    return camera_info_manager_->isCalibrated() ? img_rect_pub_->getNumSubscribers() : 0;
  }

  uint32_t ArenaCameraNode::getNumSubscribers() const
  {
    return img_raw_pub_.getNumSubscribers() + img_rect_pub_->getNumSubscribers()
                                            + img_scaled_pub_->getNumSubscribers();
  }

  void ArenaCameraNode::setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg)
  {
    // http://www.ros.org/reps/rep-0104.html

    // If the camera is uncalibrated, matrices D, K, R, and P should be left
    // zeroed out. In particular, users may assume that K[0] == 0.0 indicates
    // an uncalibrated camera.
    cam_info_msg.header.frame_id = arena_camera_parameter_set_.cameraFrame();
    cam_info_msg.header.stamp = ros::Time::now();

    // Image dimensions with which the camera was calibrated, usually the full
    // camera resolution in pixels. They remain fixed, even if binning is used.
    cam_info_msg.height = Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height");
    cam_info_msg.width = Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width");

    // The distortion model used. Supported models are listed in
    // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
    // simple model of radial and tangential distortion - is sufficient.
    // Empty D and distortion_model indicate that CameraInfo cannot be used to
    // rectify points or images, either because the camera is not calibrated or
    // because the rectified image was produced using an unsupported
    // distortion model, e.g. the proprietary one used by Bumblebee cameras.
    cam_info_msg.distortion_model = "";

    // Distortion parameters, the size depending on the distortion model.
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    cam_info_msg.D = std::vector<double>(5, 0.);

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    // Projects 3D points in the camera coordinate frame to 2D pixel
    // coordinates using the focal lengths (fx, fy) and principal point
    // (cx, cy).
    cam_info_msg.K.assign(0.0);

    // Rectification matrix (stereo cameras only).
    // A rotation matrix aligning the camera coordinate system to the ideal
    // stereo image plane so epipolar lines in both stereo images are parallel.
    cam_info_msg.R.assign(0.0);

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    // By convention, this matrix specifies the intrinsic (camera) matrix of
    // the processed (rectified) image. That is, the left 3x3 portion is the
    // normal intrinsic matrix for the rectified image.
    // It projects 3D points in the camera coordinate frame to 2D pixel
    // coordinates using the focal lengths (fx', fy') and principal point
    // (cx', cy') - these may differ from the values in K.
    // For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
    // also have R = the identity matrix and P[1:3, 1:3] = K.
    // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    // position of the optical center of the second camera in the first
    // camera's frame. We assume Tz = 0, i.e. both cameras are in the same
    // stereo image plane. The first camera always has Tx = Ty = 0. For the
    // right (second) camera of a horizontal stereo pair, Ty = 0 and
    // Tx = -fx' * B, where B is the baseline between the cameras.
    // Given a 3D point [X Y Z]', the projection (x, y) of the point onto the
    // rectified image is given by:
    // [u v w]' = P * [X Y Z 1]'
    //        x = u / w
    //        y = v / w
    // This holds for both images of a stereo pair.
    cam_info_msg.P.assign(0.0);

    // Binning refers to any camera setting which combines rectangular
    // neighborhoods of pixels into larger 'super-pixels'. It reduces the
    // resolution of the output image to
    // (width / binning_x) x (height / binning_y).
    // The default values binning_x = binning_y = 0 are considered the
    // same as binning_x = binning_y = 1 (no subsampling).
    cam_info_msg.binning_x = 0; // currentBinningX();
    cam_info_msg.binning_y = 0; // currentBinningY();

    // Region of interest (subwindow of full camera resolution), given in full
    // resolution (unbinned) image coordinates. A particular ROI always denotes
    // the same window of pixels on the camera sensor, regardless of binning
    // settings.
    // The default setting of ROI (all values 0) is considered the same
    // as full resolution (roi.width = width, roi.height = height).
    cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
    cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
  }

  bool setROIValue(const sensor_msgs::RegionOfInterest target_roi,
                    sensor_msgs::RegionOfInterest& reached_roi)
  {
    try
    {
      GenApi::CIntegerPtr pWidth = pDevice_->GetNodeMap()->GetNode("Width");
      GenApi::CIntegerPtr pHeight = pDevice_->GetNodeMap()->GetNode("Height");
      GenApi::CIntegerPtr pOffsetX = pDevice_->GetNodeMap()->GetNode("OffsetX");
      GenApi::CIntegerPtr pOffsetY = pDevice_->GetNodeMap()->GetNode("OffsetY");

      if (GenApi::IsWritable(pWidth) && GenApi::IsWritable(pHeight) &&
          GenApi::IsWritable(pOffsetX) && GenApi::IsWritable(pOffsetY))
      {
        size_t width_to_set = target_roi.width;
        size_t height_to_set = target_roi.height;
        size_t x_offset_to_set = target_roi.x_offset;
        size_t y_offset_to_set = target_roi.y_offset;

        if (x_offset_to_set < pOffsetX->GetMin())
        {
          ROS_WARN_STREAM("Desired horizontal offset (" << x_offset_to_set << ") is unreachable! "
                          << "Setting horizontal offset to the lower limit ("
                          << pOffsetX->GetMin() << ").");
          
          x_offset_to_set = pOffsetX->GetMin();
        }
        else if (x_offset_to_set > pOffsetX->GetMax())
        {
          ROS_WARN_STREAM("Desired horizontal offset (" << x_offset_to_set << ") is unreachable! "
                          << "Setting horizontal offset to the upper limit ("
                          << pOffsetX->GetMax() << ").");
          
          x_offset_to_set = pOffsetX->GetMax();
        }

        if (y_offset_to_set < pOffsetY->GetMin())
        {
          ROS_WARN_STREAM("Desired vertical offset (" << y_offset_to_set << ") is unreachable! "
                          << "Setting vertical offset to the lower limit ("
                          << pOffsetY->GetMin() << ").");
          
          y_offset_to_set = pOffsetY->GetMin();
        }
        else if (y_offset_to_set > pOffsetY->GetMax())
        {
          ROS_WARN_STREAM("Desired vertical offset (" << y_offset_to_set << ") is unreachable! "
                          << "Setting vertical offset to the upper limit ("
                          << pOffsetY->GetMax() << ").");
          
          y_offset_to_set = pOffsetY->GetMax();
        }

        if (width_to_set < pWidth->GetMin())
        {
          ROS_WARN_STREAM("Desired width (" << width_to_set << ") is unreachable! Setting width "
                          << "to the lower limit (" << pWidth->GetMin() << ").");
          
          width_to_set = pWidth->GetMin();
        }
        else if (width_to_set > pWidth->GetMax() - x_offset_to_set)
        {
          ROS_WARN_STREAM("Desired width (" << width_to_set << ") is unreachable! Setting width "
                          << "to the upper limit (" << pWidth->GetMax() - x_offset_to_set << ").");
          
          width_to_set = pWidth->GetMax() - x_offset_to_set;
        }

        if (height_to_set < pHeight->GetMin())
        {
          ROS_WARN_STREAM("Desired height (" << height_to_set << ") is unreachable! Setting "
                          << "height to the lower limit (" << pHeight->GetMin() << ").");
          
          height_to_set = pHeight->GetMin();
        }
        else if (height_to_set > pHeight->GetMax() - y_offset_to_set)
        {
          ROS_WARN_STREAM("Desired height (" << height_to_set << ") is unreachable! Setting "
                          << "height to the upper limit (" << pWidth->GetMax() - x_offset_to_set
                          << ").");
          
          height_to_set = pHeight->GetMax() - y_offset_to_set;
        }

        pWidth->SetValue(width_to_set);
        pHeight->SetValue(height_to_set);
        pOffsetX->SetValue(x_offset_to_set);
        pOffsetY->SetValue(y_offset_to_set);
        
        reached_roi = currentROI();
      }
      else
      {
        ROS_WARN_STREAM("Camera does not support changing the ROI. Will keep the current "
                        << "settings.");
        
        reached_roi = currentROI();
      }
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while changing the ROI: " << e.GetDescription());

      return false;
    }

    return true;
  }
  
  bool ArenaCameraNode::setROI(const sensor_msgs::RegionOfInterest target_roi,
                                sensor_msgs::RegionOfInterest& reached_roi)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    
    if (!setROIValue(target_roi, reached_roi))
    {
      // Retry until timeout.
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(2.0));

      while (ros::ok())
      {
        if (setROIValue(target_roi, reached_roi))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in setROI(): unable to set the target ROI before timeout.");

          return false;
        }

        r.sleep();
      }
    }

    return true;
  }

  bool ArenaCameraNode::setROICallback(camera_control_msgs::SetROI::Request& req,
                                        camera_control_msgs::SetROI::Response& res)
  {
    res.success = setROI(req.target_roi, res.reached_roi);
    
    return true;
  }

  bool setBinningXValue(const size_t& target_binning_x, size_t& reached_binning_x)
  {
    try
    { 
      GenApi::CIntegerPtr pBinningHorizontal = pDevice_->GetNodeMap()->GetNode("BinningHorizontal");
      
      if (GenApi::IsWritable(pBinningHorizontal))
      {
        size_t binning_x_to_set = target_binning_x;
        
        if (binning_x_to_set < pBinningHorizontal->GetMin())
        {
          ROS_WARN_STREAM("Desired horizontal binning (" << binning_x_to_set << ") unreachable! "
                          << "Setting it to the lower limit (" << pBinningHorizontal->GetMin()
                          << ").");
          
          binning_x_to_set = pBinningHorizontal->GetMin();
        }
        else if (binning_x_to_set > pBinningHorizontal->GetMax())
        {
          ROS_WARN_STREAM("Desired horizontal binning (" << binning_x_to_set << ") unreachable! "
                          << "Setting it to the upper limit (" << pBinningHorizontal->GetMax()
                          << ").");
          
          binning_x_to_set = pBinningHorizontal->GetMax();
        }

        pBinningHorizontal->SetValue(binning_x_to_set);
        reached_binning_x = currentBinningX();

        ROS_INFO_STREAM("Setting horizontal binning to " << binning_x_to_set << ", reached: "
                        << reached_binning_x << ".");
      }
      else
      {
        ROS_WARN_STREAM("Binning settings cannot be changed while camera is in operation, or camera "
                        << "does not support binning. Current settings will be kept.");

        reached_binning_x = currentBinningX();
      }
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while setting horizontal binning to "
                        << target_binning_x << ": " << e.GetDescription());
      
      return false;
    }
    
    return true;
  }

  bool ArenaCameraNode::setBinningX(const size_t& target_binning_x, size_t& reached_binning_x)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

    if (!setBinningXValue(target_binning_x, reached_binning_x))
    {
      // Retry until timeout.
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
      
      while (ros::ok())
      {
        if (setBinningXValue(target_binning_x, reached_binning_x))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in setBinningX(): Unable to set horizontal binning before "
                            << "timeout.");

          return false;
        }

        r.sleep();
      }
    }

    return true;
  }

  bool setBinningYValue(const size_t& target_binning_y, size_t& reached_binning_y)
  {
    try
    {
      GenApi::CIntegerPtr pBinningVertical = pDevice_->GetNodeMap()->GetNode("BinningVertical");
      
      if (GenApi::IsWritable(pBinningVertical))
      {
        size_t binning_y_to_set = target_binning_y;
        
        if (binning_y_to_set < pBinningVertical->GetMin())
        {
          ROS_WARN_STREAM("Desired vertical binning (" << binning_y_to_set << ") unreachable! "
                          << "Setting it to the lower limit (" << pBinningVertical->GetMin()
                          << ").");

          binning_y_to_set = pBinningVertical->GetMin();
        }
        else if (binning_y_to_set > pBinningVertical->GetMax())
        {
          ROS_WARN_STREAM("Desired vertical binning (" << binning_y_to_set << ") unreachable! "
                          << "Setting it to the upper limit (" << pBinningVertical->GetMax()
                          << ").");
          
          binning_y_to_set = pBinningVertical->GetMax();
        }

        pBinningVertical->SetValue(binning_y_to_set);
        reached_binning_y = currentBinningY();

        ROS_INFO_STREAM("Setting vertical binning to " << binning_y_to_set << ", reached: "
                      << reached_binning_y << ".");
      }
      else
      {
        ROS_WARN_STREAM("Binning settings cannot be changed while camera is in operation, or camera "
                        << "does not support binning. Current settings will be kept.");

        reached_binning_y = currentBinningY();
      }
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while setting vertical binning to "
                        << target_binning_y << ": " << e.GetDescription());

      return false;
    }

    return true;
  }

  bool ArenaCameraNode::setBinningY(const size_t& target_binning_y, size_t& reached_binning_y)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

    if (!setBinningYValue(target_binning_y, reached_binning_y))
    {
      // Retry until timeout.
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
      
      while (ros::ok())
      {
        if (setBinningYValue(target_binning_y, reached_binning_y))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in setBinningY(): Unable to set vertical binning before "
                            << "timeout.");

          return false;
        }

        r.sleep();
      }
    }

    return true;
  }

  bool ArenaCameraNode::setBinningCallback(camera_control_msgs::SetBinning::Request& req,
                                            camera_control_msgs::SetBinning::Response& res)
  {
    GenApi::CEnumerationPtr pBinningSelector = pDevice_->GetNodeMap()->GetNode("BinningSelector");

    if (GenApi::IsWritable(pBinningSelector))
    {
      if (arena_camera_parameter_set_.sensor_binning_)
      {
        Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                "BinningSelector",
                                                "Sensor");

        ROS_INFO_STREAM("Using sensor binning.");
      }
      else
      {
        Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                "BinningSelector",
                                                "Digital");

        ROS_INFO_STREAM("Using digital binning.");
      }
    }
    else
    {
      ROS_WARN_STREAM("Cannot change sensor binning mode.");
    }
    
    size_t reached_binning_x, reached_binning_y;
    
    bool success_x = setBinningX(req.target_binning_x, reached_binning_x);
    bool success_y = setBinningY(req.target_binning_y, reached_binning_y);
    
    res.reached_binning_x = static_cast<uint32_t>(reached_binning_x);
    res.reached_binning_y = static_cast<uint32_t>(reached_binning_y);
    res.success = success_x && success_y;
    
    return true;
  }

  bool ArenaCameraNode::setExposureValue(const float& target_exposure, float& reached_exposure)
  {
    try
    {
      Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Off");

      GenApi::CFloatPtr pExposureTime = pDevice_->GetNodeMap()->GetNode("ExposureTime");

      float exposure_to_set = target_exposure;
      
      if (exposure_to_set < pExposureTime->GetMin())
      {
        ROS_WARN_STREAM("Desired exposure time (" << exposure_to_set << ") unreachable! Setting "
                        << "it to the lower limit (" << pExposureTime->GetMin() << ").");

        exposure_to_set = pExposureTime->GetMin() + 0.01;
      }
      else if (exposure_to_set > pExposureTime->GetMax())
      {
        ROS_WARN_STREAM("Desired exposure time (" << exposure_to_set << ") unreachable! Setting "
                        << "it to the upper limit (" << pExposureTime->GetMax() << ").");

        exposure_to_set = pExposureTime->GetMax() - 0.01;
      }

      pExposureTime->SetValue(exposure_to_set);
      reached_exposure = currentExposure();

      ROS_INFO_STREAM("Setting exposure time to " << exposure_to_set << " us, reached: "
                      << reached_exposure << " us.");
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while setting exposure time to "
                        << target_exposure << ": " << e.GetDescription());

      return false;
    }

    return true;
  }

  bool ArenaCameraNode::setExposure(const float& target_exposure, float& reached_exposure)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

    if (ArenaCameraNode::setExposureValue(target_exposure, reached_exposure))
    {
      return true;
    }
    else
    {
      // Retry until timeout.
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
      
      while (ros::ok())
      {
        if (ArenaCameraNode::setExposureValue(target_exposure, reached_exposure))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in setExposure(): Unable to set exposure time before timeout.");

          return false;
        }

        r.sleep();
      }

      return true;
    }
  }

  bool ArenaCameraNode::setExposureCallback(camera_control_msgs::SetExposure::Request& req,
                                            camera_control_msgs::SetExposure::Response& res)
  {
    res.success = setExposure(req.target_exposure, res.reached_exposure);
    
    return true;
  }

  bool ArenaCameraNode::setGainValue(const float& target_gain, float& reached_gain)
  {
    try
    {
      Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Off");

      GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");
      
      float gain_fraction = target_gain;
      
      if (gain_fraction < 0)
      {
        ROS_WARN_STREAM("Desired gain fraction (" << gain_fraction << ") unreachable! Setting "
                        << "it to the lower limit (0).");
        
        gain_fraction = 0;
      }
      else if (gain_fraction > 1)
      {
        ROS_WARN_STREAM("Desired gain fraction (" << gain_fraction << ") unreachable! Setting "
                        << "it to the upper limit (1).");

        gain_fraction = 1;
      }

      float gain_to_set = pGain->GetMin() + gain_fraction * (pGain->GetMax() - pGain->GetMin());
      
      pGain->SetValue(gain_to_set);
      reached_gain = currentGain();

      ROS_INFO_STREAM("Setting gain to " << gain_to_set << " dB, reached: " << reached_gain
                      << " dB.");
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while setting gain to " << target_gain << ": "
                        << e.GetDescription());
      
      return false;
    }
    
    return true;
  }

  bool ArenaCameraNode::setGain(const float& target_gain, float& reached_gain)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

    if (ArenaCameraNode::setGainValue(target_gain, reached_gain))
    {
      return true;
    }
    else
    {
      // Retry until timeout
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
      
      while (ros::ok())
      {
        if (ArenaCameraNode::setGainValue(target_gain, reached_gain))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in setGain(): Unable to set gain before timeout.");

          return false;
        }

        r.sleep();
      }

      return true;
    }
  }

  bool ArenaCameraNode::setGainCallback(camera_control_msgs::SetGain::Request& req,
                                        camera_control_msgs::SetGain::Response& res)
  {
    res.success = setGain(req.target_gain, res.reached_gain);
    
    return true;
  }

  bool ArenaCameraNode::setGammaValue(const float& target_gamma, float& reached_gamma)
  {
    GenApi::CBooleanPtr pGammaEnable = pDevice_->GetNodeMap()->GetNode("GammaEnable");
    
    if (pGammaEnable && GenApi::IsWritable(pGammaEnable))
    {
      pGammaEnable->SetValue(true);
    }

    GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");
    
    if (!pGamma || !GenApi::IsWritable(pGamma))
    {
      ROS_WARN_STREAM("Gamma settings cannot be changed while camera is in operation, or camera "
                        << "does not support gamma correction. Current settings will be kept.");
      
      reached_gamma = -1;
      
      return true;
    }
    else
    {
      try
      {
        float gamma_to_set = target_gamma;
        
        if (gamma_to_set < pGamma->GetMin())
        {
          ROS_WARN_STREAM("Desired gamma (" << gamma_to_set << ") unreachable! Setting it to the "
                          << "lower limit (" << pGamma->GetMin() << ").");

          gamma_to_set = pGamma->GetMin();
        }
        else if (gamma_to_set > pGamma->GetMax())
        {
          ROS_WARN_STREAM("Desired gamma (" << gamma_to_set << ") unreachable! Setting it to the "
                          << "upper limit (" << pGamma->GetMax() << ").");

          gamma_to_set = pGamma->GetMax();
        }

        pGamma->SetValue(gamma_to_set);
        reached_gamma = currentGamma();

        ROS_INFO_STREAM("Setting gamma to " << gamma_to_set << ", reached: " << reached_gamma
                        << ".");
      }
      catch (const GenICam::GenericException& e)
      {
        ROS_ERROR_STREAM("An exception occurred while setting gamma to " << target_gamma << ": "
                          << e.GetDescription());
        
        return false;
      }
    }
    
    return true;
  }

  bool ArenaCameraNode::setGamma(const float& target_gamma, float& reached_gamma)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    
    if (ArenaCameraNode::setGammaValue(target_gamma, reached_gamma))
    {
      return true;
    }
    else
    {
      // Retry until timeout.
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
      
      while (ros::ok())
      {
        if (ArenaCameraNode::setGammaValue(target_gamma, reached_gamma))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in setGamma(): Unable to set gamma before timeout.");

          return false;
        }

        r.sleep();
      }

      return true;
    }
  }

  bool ArenaCameraNode::setGammaCallback(camera_control_msgs::SetGamma::Request& req,
                                          camera_control_msgs::SetGamma::Response& res)
  {
    res.success = setGamma(req.target_gamma, res.reached_gamma);
    
    return true;
  }

  bool ArenaCameraNode::setBrightnessValue(const int& target_brightness, int& reached_brightness,
                                            const bool& exposure_auto, const bool& gain_auto)
  {
    try
    {
      GenApi::CIntegerPtr pBrightness = pDevice_->GetNodeMap()->GetNode("TargetBrightness");
      
      if (!exposure_auto && !gain_auto)
      {
        ROS_WARN_STREAM("Cannot change brightness because both exposure time and gain have been "
                        << "manually set. Turn either auto exposure or auto gain on.");

        reached_brightness = pBrightness->GetValue();

        return true;
      }
      else
      {
        if (exposure_auto)
        {
          Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto",
                                                  "Continuous");
        }
        else
        {
          Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Off");
        }

        if (gain_auto)
        {
          Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Continuous");
        }
        else
        {
          Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Off");
        }
      }
      
      int brightness_to_set = target_brightness;
      
      if (brightness_to_set < pBrightness->GetMin())
      {
        ROS_WARN_STREAM("Desired brightness (" << brightness_to_set << ") unreachable! Setting "
                        << "it to the lower limit (" << pBrightness->GetMin() << ").");

        brightness_to_set = pBrightness->GetMin();
      }
      else if (brightness_to_set > pBrightness->GetMax())
      {
        ROS_WARN_STREAM("Desired brightness (" << brightness_to_set << ") unreachable! Setting "
                        << "it to the upper limit (" << pBrightness->GetMax() << ").");

        brightness_to_set = pBrightness->GetMax();
      }

      pBrightness->SetValue(brightness_to_set);
      reached_brightness = pBrightness->GetValue();

      ROS_INFO_STREAM("Setting brightness to " << brightness_to_set << ", reached: "
                      << reached_brightness << ".");
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while setting brightness to " << target_brightness
                        << ": " << e.GetDescription());
      
      return false;
    }
    
    return true;
  }
  
  bool ArenaCameraNode::setBrightness(const int& target_brightness, int& reached_brightness,
                                      const bool& exposure_auto, const bool& gain_auto)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    
    if (ArenaCameraNode::setBrightnessValue(target_brightness, reached_brightness,
                                            exposure_auto, gain_auto))
    {
      return true;
    }
    else
    {
      // Retry until timeout.
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
      
      while (ros::ok())
      {
        if (ArenaCameraNode::setBrightnessValue(target_brightness, reached_brightness,
                                                exposure_auto, gain_auto))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in setBrightness(): Unable to set brightness before timeout.");

          return false;
        }

        r.sleep();
      }

      return true;
    }
  }
  
  bool ArenaCameraNode::setBrightnessCallback(camera_control_msgs::SetBrightness::Request& req,
                                              camera_control_msgs::SetBrightness::Response& res)
  {
    res.success = setBrightness(req.target_brightness, res.reached_brightness,
                                req.exposure_auto, req.gain_auto);

    ros::Duration(2.0).sleep();
    
    res.reached_exposure_time = currentExposure();
    res.reached_gain_value = currentGain();
    
    return true;
  }

  bool ArenaCameraNode::setSleepingCallback(camera_control_msgs::SetSleeping::Request& req,
                                            camera_control_msgs::SetSleeping::Response& res)
  {
    is_sleeping_ = req.set_sleeping;

    if (is_sleeping_)
    {
      ROS_WARN_STREAM("Putting the camera to sleep.");
    }
    else
    {
      ROS_WARN_STREAM("Waking up the camera.");
    }

    res.success = true;
    
    return true;
  }

  bool ArenaCameraNode::getPropertiesValue(camera_control_msgs::GetCamProperties::Response& res)
  {
    try
    {
      auto pNodeMap_ = pDevice_->GetNodeMap();
      
      // The Arena interface seems unable to provide the latest values of
      // exposure time and gain, which may automatically change based on lighting
      // conditions. Hence we first change the status of auto exposure and auto
      // gain, then revert them back to their original values, to get the latest
      // exposure time and gain values. 

      GenApi::CEnumerationPtr pExposureAuto = pNodeMap_->GetNode("ExposureAuto");
      GenApi::CEnumerationPtr pGainAuto = pNodeMap_->GetNode("GainAuto");

      auto exposure_auto = pExposureAuto->GetIntValue();
      auto gain_auto = pGainAuto->GetIntValue();

      if (GenApi::IsWritable(pExposureAuto))
      {
        pExposureAuto->SetIntValue(2 - exposure_auto);
        pExposureAuto->SetIntValue(exposure_auto);
      }

      if (GenApi::IsWritable(pGainAuto))
      {
        pGainAuto->SetIntValue(2 - gain_auto);
        pGainAuto->SetIntValue(gain_auto);
      }    
      
      // Get sleeping status.
      res.is_sleeping = isSleeping();

      // Get device user ID.
      GenApi::CStringPtr pUserID = pNodeMap_->GetNode("DeviceUserID");
      
      if (GenApi::IsReadable(pUserID))
      {
        res.device_user_id = pUserID->GetValue();
      }

      // Get horizontal binning properties.
      GenApi::CIntegerPtr pBinningHorizontal = pNodeMap_->GetNode("BinningHorizontal");

      if (GenApi::IsReadable(pBinningHorizontal))
      {
        res.min_binning_x = pBinningHorizontal->GetMin();
        res.max_binning_x = pBinningHorizontal->GetMax();
        res.current_binning_x = pBinningHorizontal->GetValue();
      }

      // Get vertical binning properties.
      GenApi::CIntegerPtr pBinningVertical = pNodeMap_->GetNode("BinningVertical");

      if (GenApi::IsReadable(pBinningVertical))
      {
        res.min_binning_y = pBinningVertical->GetMin();
        res.max_binning_y = pBinningVertical->GetMax();
        res.current_binning_y = pBinningVertical->GetValue();
      }

      // Get current and maximum frame rate.
      GenApi::CFloatPtr pFrameRate = pNodeMap_->GetNode("AcquisitionFrameRate");

      if (GenApi::IsReadable(pFrameRate))
      {
        res.max_framerate = pFrameRate->GetMax();
        res.current_framerate = pFrameRate->GetValue();
      }

      // Get exposure time properties.
      GenApi::CFloatPtr pExposureTime = pNodeMap_->GetNode("ExposureTime");

      if (GenApi::IsReadable(pExposureTime))
      {
        res.min_exposure = pExposureTime->GetMin();
        res.max_exposure = pExposureTime->GetMax();
        res.current_exposure = pExposureTime->GetValue();
      }

      // Get gain properties.
      GenApi::CFloatPtr pGain = pNodeMap_->GetNode("Gain");

      if (GenApi::IsReadable(pGain))
      {
        res.min_gain_in_cam_units = pGain->GetMin();
        res.max_gain_in_cam_units = pGain->GetMax();
        res.current_gain_in_cam_units = pGain->GetValue();

        res.min_gain = 0;
        res.max_gain = 1;
        res.current_gain = (pGain->GetValue() - pGain->GetMin()) / (pGain->GetMax() - pGain->GetMin());
      }

      // Get gamma properties.
      GenApi::CFloatPtr pGamma = pNodeMap_->GetNode("Gamma");

      if (GenApi::IsReadable(pGamma))
      {
        res.min_gamma = pGamma->GetMin();
        res.max_gamma = pGamma->GetMax();
        res.current_gamma = pGamma->GetValue();
      }

      // Get brightness properties.
      GenApi::CIntegerPtr pBrightness = pNodeMap_->GetNode("TargetBrightness");

      if (GenApi::IsReadable(pBrightness))
      {
        res.min_brightness = pBrightness->GetMin();
        res.max_brightness = pBrightness->GetMax();
        res.current_brightness = pBrightness->GetValue();
      }

      res.gain_auto = (gain_auto != 0);
      res.exposure_auto = (exposure_auto != 0);
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM("An exception occurred while getting camera properties: "
                        << e.GetDescription());
      
      return false;
    }
    
    return true;
  }

  bool ArenaCameraNode::getProperties(camera_control_msgs::GetCamProperties::Response& res)
  {
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    
    if (ArenaCameraNode::getPropertiesValue(res))
    {
      return true;
    }
    else
    {
      // Retry until timeout.
      ros::Rate r(10.0);
      ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
      
      while (ros::ok())
      {
        if (ArenaCameraNode::getPropertiesValue(res))
        {
          break;
        }
        if (ros::Time::now() > timeout)
        {
          ROS_ERROR_STREAM("Error in getProperties(): Unable to get camera properties before "
                            << "timeout.");

          return false;
        }

        r.sleep();
      }

      return true;
    }
  }

  bool ArenaCameraNode::getPropertiesCallback(camera_control_msgs::GetCamProperties::Request& req,
                                              camera_control_msgs::GetCamProperties::Response& res)
  {
    res.success = getProperties(res);
    
    return true;
  }

  bool ArenaCameraNode::isSleeping()
  {
    return is_sleeping_;
  }

  ArenaCameraNode::~ArenaCameraNode()
  {
    if (pDevice_ != nullptr)
    {
      pSystem_->DestroyDevice(pDevice_);
    }
    if (pSystem_ != nullptr)
    {
      Arena::CloseSystem(pSystem_);
    }
    if (it_)
    {
      delete it_;
      
      it_ = nullptr;
    }
    if (grab_imgs_rect_as_)
    {
      grab_imgs_rect_as_->shutdown();
      
      delete grab_imgs_rect_as_;
      
      grab_imgs_rect_as_ = nullptr;
    }
    if (img_rect_pub_)
    {
      delete img_rect_pub_;
      
      img_rect_pub_ = nullptr;
    }
    if (cv_bridge_img_rect_)
    {
      delete cv_bridge_img_rect_;
      
      cv_bridge_img_rect_ = nullptr;
    }
    if (img_scaled_pub_)
    {
      delete img_scaled_pub_;
      
      img_scaled_pub_ = nullptr;
    }
    if (discovery_pub_)
    {
      delete discovery_pub_;
      
      discovery_pub_ = nullptr;
    }
    if (pinhole_model_)
    {
      delete pinhole_model_;
      
      pinhole_model_ = nullptr;
    }
  }
}  // namespace arena_camera