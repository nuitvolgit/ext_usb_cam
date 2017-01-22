/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
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
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;

  image_transport::CameraPublisher cam_pub_;

  sensor_msgs::CameraInfoPtr ci_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;

  int framerate_;

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }

  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() : node_("~")
  {
    // parameters
    std::string video_device_name, io_method_name, pixel_format_name, field_order_name,
        camera_name, camera_info_url;
    int image_width, image_height, exposure, brightness, contrast, saturation, sharpness,
        focus, white_balance, gain;
    bool autofocus, autoexposure, auto_white_balance, bayer_to_grey;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo;

    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    cam_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name, std::string("/dev/video0"));
    node_.param("brightness", brightness, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name, std::string("mmap"));
    node_.param("image_width", image_width, 640);
    node_.param("image_height", image_height, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24, grbg
    node_.param("pixel_format", pixel_format_name, std::string("yuyv"));
    // In the case where input image is grgb(=bayer_grbg8) and wants to conver to grey
    node_.param("bayer_to_grey", bayer_to_grey, false);
    // possible values: any, none, top, bottom, interlaced, tb, bt, alternate    
    node_.param("field_order", field_order_name, std::string("none"));

    // enable/disable autofocus
    node_.param("autofocus", autofocus, false);
    node_.param("focus", focus, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure, true);
    node_.param("exposure", exposure, 100);
    node_.param("gain", gain, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance, true);
    node_.param("white_balance", white_balance, 4000);
    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url, std::string(""));
    cinfo.reset(new camera_info_manager::CameraInfoManager(node_, camera_name, camera_info_url));

    // check for default camera info
    if (!cinfo->isCalibrated()) {
      cinfo->setCameraName(video_device_name);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width;
      camera_info.height = image_height;
      cinfo->setCameraInfo(camera_info);
    }
    ci_.reset(new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));
    ci_->header.frame_id = img_.header.frame_id;

    // create Services
    // >> rosservice call /camera/start_capture
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    // >> rosservice call /camera/stop_capture
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    ROS_WARN("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name.c_str(), video_device_name.c_str(),
        image_width, image_height, io_method_name.c_str(), pixel_format_name.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN) {
      ROS_FATAL("Unknown IO method '%s'", io_method_name.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name.c_str());
      node_.shutdown();
      return;
    }

    // set the field order
    UsbCam::field_order field = UsbCam::field_order_from_string(field_order_name);
    if (field == UsbCam::FIELD_UNKNOWN)
    {
      ROS_FATAL("Unknown field order '%s'", field_order_name.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name.c_str(), io_method, pixel_format, field, image_width,
         image_height, framerate_, bayer_to_grey);

    // set camera parameters
    if (brightness >= 0)
      cam_.set_v4l_parameter("brightness", brightness);
    if (contrast >= 0)
      cam_.set_v4l_parameter("contrast", contrast);
    if (saturation >= 0)
      cam_.set_v4l_parameter("saturation", saturation);
    if (sharpness >= 0)
      cam_.set_v4l_parameter("sharpness", sharpness);
    if (gain >= 0)
      cam_.set_v4l_parameter("gain", gain);

    // check auto white balance
    if (auto_white_balance) {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    } else {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance);
    }

    // check auto exposure
    if (!autoexposure) {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure);
    }

    // check auto focus
    if (autofocus) {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focusauto", 1);
    } else {
      cam_.set_v4l_parameter("focusauto", 0);
      if (focus >= 0)
        cam_.set_v4l_parameter("focusabsolute", focus);
    }
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // stamp the image    
    ci_->header.stamp = img_.header.stamp;

    // publish the camera
    cam_pub_.publish(img_, *ci_);


    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok()) {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }

};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
