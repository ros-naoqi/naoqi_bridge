/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2009, 2010 Jack O'Quin, Patrick Beeson
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <sstream>

#include <boost/format.hpp>

#include <driver_base/SensorLevels.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

// Aldebaran includes
#include <alproxies/almemoryproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alvalue/alvalue.h>
#include <alcommon/altoolsmain.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/alproxy.h>
#include <alproxies/alproxies.h>
#include <alcommon/almodule.h>



#include "naoqi_sensors/NaoqiCameraConfig.h"

#include "naoqi_camera.h"
/** @file

@brief ROS driver for IIDC-compatible IEEE 1394 digital cameras.

This is a ROS driver for 1394 cameras, using libdc1394.  It can be
instantiated as either a node or a nodelet.  It is written with with
minimal dependencies, intended to fill a role in the ROS image
pipeline similar to the other ROS camera drivers.

@par Advertises

 - @b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images

 - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each image.

*/

using namespace std;
using namespace AL;

namespace naoqicamera_driver
{
  // some convenience typedefs
  typedef naoqicamera::NaoqiCameraConfig Config;
  typedef driver_base::Driver Driver;
  typedef driver_base::SensorLevels Levels;

  NaoqiCameraDriver::NaoqiCameraDriver(int argc, char ** argv,
                                   ros::NodeHandle priv_nh,
                                   ros::NodeHandle camera_nh):
    NaoqiNode(argc, argv),
    state_(Driver::CLOSED),
    priv_nh_(priv_nh),
    camera_nh_(camera_nh),
    camera_name_("camera"),
    cycle_(1.0),                        // slow poll when closed
    real_frame_rate_(30.0),
    retries_(0),
    srv_(priv_nh),
    cinfo_(new camera_info_manager::CameraInfoManager(camera_nh_)),
    calibration_matches_(true),
    it_(new image_transport::ImageTransport(camera_nh_)),
    image_pub_(it_->advertiseCamera("image_raw", 1)),
    diagnostics_(),
    topic_diagnostics_min_freq_(0.),
    topic_diagnostics_max_freq_(1000.),
    topic_diagnostics_("image_raw", diagnostics_,
                diagnostic_updater::FrequencyStatusParam
                    (&topic_diagnostics_min_freq_,
                     &topic_diagnostics_max_freq_, 0.1, 10),
                diagnostic_updater::TimeStampStatusParam())
  {
    getNaoqiParams(priv_nh);
    if ( !connectNaoQi() )
    {
      ROS_ERROR("Could not connect to NAOqi! Make sure NAOqi is running and you passed the right host/port.");
      throw naoqicamera_driver::Exception("Connection to NAOqi failed");
    }

  }

  NaoqiCameraDriver::~NaoqiCameraDriver()
  {}


  /** Get broker ip and port from ROS parameters.
   * @param nh Nodehandle used to get parameters
   *
   */
  void NaoqiCameraDriver::getNaoqiParams(ros::NodeHandle nh)
  {
    if( !nh.getParam("pip", m_pip) )
      ROS_WARN("No pip parameter specified.");
    if( !nh.getParam("pport", m_pport) )
      ROS_WARN("No pport parameter specified.");
    if( !nh.getParam("ip", m_ip) )
      ROS_DEBUG("No ip parameter specified.");
    if( !nh.getParam("port", m_port) )
      ROS_DEBUG("No port parameter specified.");

    ROS_INFO_STREAM("pip: " << m_pip);
    ROS_INFO_STREAM("pip: " << m_pport);
    ROS_INFO_STREAM("ip:" << m_ip);
    ROS_INFO_STREAM("port: " << m_port);
  }

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void NaoqiCameraDriver::closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        camera_proxy_->unsubscribe(camera_name_);
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * @post diagnostics frequency parameters set
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   *   GUID configuration parameter updated
   */
  bool NaoqiCameraDriver::openCamera(Config &newconfig)
  {
    bool success = false;

    try
    {
        camera_proxy_ = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(m_broker));

        if (camera_proxy_->getGenericProxy()->isLocal())
            ROS_INFO("nao_camera runs directly on the robot.");
        else
            ROS_WARN("nao_camera runs remotely.");
        // build a unique name for the camera that will be used to
        // store calibration informations.
        stringstream canonical_name;
        canonical_name << "nao_camera" 
                       << "_src" << newconfig.source 
                       << "_res" << newconfig.resolution;

        camera_name_ = camera_proxy_->subscribeCamera(
                                    camera_name_, 
                                    newconfig.source,
                                    newconfig.resolution,
                                    kRGBColorSpace,
                                    newconfig.frame_rate);

        if (!cinfo_->setCameraName(canonical_name.str()))
        {
            // GUID is 16 hex digits, which should be valid.
            // If not, use it for log messages anyway.
            ROS_WARN_STREAM("[" << camera_name_
                            << "] name not valid"
                            << " for camera_info_manger");
        }

        ROS_INFO_STREAM("[" << camera_name_ << "] opened: resolution "
                        << newconfig.resolution << " @"
                        << newconfig.frame_rate << " fps");
        state_ = Driver::OPENED;
        calibration_matches_ = true;
        newconfig.guid = camera_name_; // update configuration parameter
        retries_ = 0;
        success = true;
    }
    catch (const ALError& e)
    {
        state_ = Driver::CLOSED;    // since the open() failed
        if (retries_++ > 0)
          ROS_DEBUG_STREAM("[" << camera_name_
                           << "] exception opening device (retrying): "
                           << e.what());
        else
          ROS_ERROR_STREAM("[" << camera_name_
                           << "] device open failed: " << e.what());
    }

    // update diagnostics parameters
    diagnostics_.setHardwareID(camera_name_);
    double delta = newconfig.frame_rate * 0.1; // allow 10% error margin
    topic_diagnostics_min_freq_ = newconfig.frame_rate - delta;
    topic_diagnostics_max_freq_ = newconfig.frame_rate + delta;

    return success;
  }


  /** device poll */
  void NaoqiCameraDriver::poll(void)
  {
    // Do not run concurrently with reconfig().
    //
    // The mutex lock should be sufficient, but the Linux pthreads
    // implementation does not guarantee fairness, and the reconfig()
    // callback thread generally suffers from lock starvation for many
    // seconds before getting to run.  So, we avoid acquiring the lock
    // if there is a reconfig() pending.
    bool do_sleep = true;
    
    // publish images only if someone subscribe to the topic
    uint32_t nbSubscribers = image_pub_.getNumSubscribers();

    if (nbSubscribers == 0)
    {
        if (state_ == Driver::OPENED)
        {
            closeCamera();
        }
    }
    else
    {
        boost::mutex::scoped_lock scopedLock(reconfiguration_mutex_);
        if (state_ == Driver::CLOSED)
        {
            openCamera(config_);        // open with current configuration
        }
        do_sleep = (state_ == Driver::CLOSED);
        if (!do_sleep)                  // openCamera() succeeded?
        {
            // driver is open, read the next image still holding lock
            sensor_msgs::ImagePtr image(new sensor_msgs::Image);
            if (read(image))
            {
                publish(image);
                real_frame_rate_.sleep();
            }
        }
    } // release mutex lock

    // Always run the diagnostics updater: no lock required.
    diagnostics_.update();

    if (do_sleep)
      {
        // device was closed or poll is not running, sleeping avoids
        // busy wait (DO NOT hold the lock while sleeping)
        cycle_.sleep();
      }
  }

  /** Publish camera stream topics
   *
   *  @param image points to latest camera frame
   */
  void NaoqiCameraDriver::publish(const sensor_msgs::ImagePtr &image)
  {
    image->header.frame_id = frame_id_;

    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr
      ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

    // check whether CameraInfo matches current video mode
    //TODO: attention! we do not check that the camera source has not changed (top camera <-> bottom camera)
    if (image->width != ci->width || 
        image->height != ci->height)
      {
        // image size does not match: publish a matching uncalibrated
        // CameraInfo instead
        if (calibration_matches_)
          {
            // warn user once
            calibration_matches_ = false;
            ROS_WARN_STREAM("[" << camera_name_
                            << "] calibration does not match video mode "
                            << "(publishing uncalibrated data)");
          }
        ci.reset(new sensor_msgs::CameraInfo());
        ci->height = image->height;
        ci->width = image->width;
      }
    else if (!calibration_matches_)
      {
        // calibration OK now
        calibration_matches_ = true;
        ROS_WARN_STREAM("[" << camera_name_
                        << "] calibration matches video mode now");
      }

    ci->header.frame_id = frame_id_;
    ci->header.stamp = image->header.stamp;

    // Publish via image_transport
    image_pub_.publish(image, ci);

    // Notify diagnostics that a message has been published. That will
    // generate a warning if messages are not published at nearly the
    // configured frame_rate.
    topic_diagnostics_.tick(image->header.stamp);
  }

  /** Read camera data.
   *
   * @param image points to camera Image message
   * @return true if successful, with image filled in
   */
  bool NaoqiCameraDriver::read(sensor_msgs::ImagePtr &image)
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");

        //TODO: support local image access. This first suppose
        // to write a MAOqi 'local' module.
        // cf: http://www.aldebaran-robotics.com/documentation/dev/cpp/tutos/create_module.html#how-to-create-a-local-module
        ALValue al_image = camera_proxy_->getImageRemote(camera_name_);

        if (config_.use_ros_time)
            image->header.stamp = ros::Time::now();
        else { 
            // use NAOqi timestamp
            image->header.stamp = ros::Time(((double) al_image[4] / 1000000.0) + (double) al_image[5]);
        }

        image->width = (int) al_image[0];
        image->height = (int) al_image[1];
        image->step = image->width * (int) al_image[2];
        image->encoding = sensor_msgs::image_encodings::RGB8;

        int image_size = image->height * image->step;
        image->data.resize(image_size);

        memcpy(&(image->data)[0], 
                al_image[6].GetBinary(),
                image_size);

        camera_proxy_->releaseImage(camera_name_);

        success = true;
        ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
      }
    catch (naoqicamera_driver::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void NaoqiCameraDriver::reconfig(Config &newconfig, uint32_t level)
  {
    // Do not run concurrently with poll().
    boost::mutex::scoped_lock scopedLock(reconfiguration_mutex_);
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.source == kTopCamera)
      frame_id_ = "CameraTop_frame";
    else if (newconfig.source == kBottomCamera)
      frame_id_ = "CameraBottom_frame";
    else {
        ROS_ERROR("Unknown video source! (neither top nor bottom camera).");
        frame_id_ = "camera";
    }

    string tf_prefix = tf::getPrefixParam(priv_nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    frame_id_ = tf::resolve(tf_prefix, frame_id_);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
    {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
    }

    if (config_.camera_info_url != newconfig.camera_info_url)
      {
        // set the new URL and load CameraInfo (if any) from it
        if (cinfo_->validateURL(newconfig.camera_info_url))
          {
            cinfo_->loadCameraInfo(newconfig.camera_info_url);
          }
        else
          {
            // new URL not valid, use the old one
            newconfig.camera_info_url = config_.camera_info_url;
          }
      }

    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
    {

        if (config_.auto_exposition != newconfig.auto_exposition)
            camera_proxy_->setParam(kCameraAutoExpositionID, newconfig.auto_exposition);

        if (config_.auto_exposure_algo != newconfig.auto_exposure_algo)
            camera_proxy_->setParam(kCameraExposureAlgorithmID, newconfig.auto_exposure_algo);

        if (config_.exposure != newconfig.exposure) {
            newconfig.auto_exposition = 0;
            camera_proxy_->setParam(kCameraAutoExpositionID, 0);
            camera_proxy_->setParam(kCameraExposureID, newconfig.exposure);
        }

        if (config_.gain != newconfig.gain) {
            newconfig.auto_exposition = 0;
            camera_proxy_->setParam(kCameraAutoExpositionID, 0);
            camera_proxy_->setParam(kCameraGainID, newconfig.gain);
        }

        if (config_.brightness != newconfig.brightness) {
            newconfig.auto_exposition = 1;
            camera_proxy_->setParam(kCameraAutoExpositionID, 1);
            camera_proxy_->setParam(kCameraBrightnessID, newconfig.brightness);
        }

        if (config_.contrast != newconfig.contrast)
            camera_proxy_->setParam(kCameraContrastID, newconfig.contrast);

        if (config_.saturation != newconfig.saturation)
            camera_proxy_->setParam(kCameraSaturationID, newconfig.saturation);

        if (config_.hue != newconfig.hue)
            camera_proxy_->setParam(kCameraHueID, newconfig.hue);

        if (config_.sharpness != newconfig.sharpness)
            camera_proxy_->setParam(kCameraSharpnessID, newconfig.sharpness);

        if (config_.auto_white_balance != newconfig.auto_white_balance)
            camera_proxy_->setParam(kCameraAutoWhiteBalanceID, newconfig.auto_white_balance);

        if (config_.white_balance != newconfig.white_balance) {
            newconfig.auto_white_balance = 0;
            camera_proxy_->setParam(kCameraAutoWhiteBalanceID, 0);
            camera_proxy_->setParam(kCameraWhiteBalanceID, newconfig.white_balance);
        }
    }

    config_ = newconfig;                // save new parameters
    real_frame_rate_ = ros::Rate(newconfig.frame_rate);

    ROS_DEBUG_STREAM("[" << camera_name_
                     << "] reconfigured: frame_id " << frame_id_
                     << ", camera_info_url " << newconfig.camera_info_url);
  }


  /** driver initialization
   *
   *  Define dynamic reconfigure callback, which gets called
   *  immediately with level 0xffffffff.  The reconfig() method will
   *  set initial parameter values, then open the device if it can.
   */
  void NaoqiCameraDriver::setup(void)
  {
    srv_.setCallback(boost::bind(&NaoqiCameraDriver::reconfig, this, _1, _2));
    ROS_INFO("Ready to publish Nao cameras. Waiting for someone to subscribe...");
  }


  /** driver termination */
  void NaoqiCameraDriver::shutdown(void)
  {
    closeCamera();
  }

}; // end namespace naocamera_driver
