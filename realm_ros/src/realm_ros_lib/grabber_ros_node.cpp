/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <realm_ros/grabber_ros_node.h>

#include <realm_core/timer.h>
#include <realm_core/loguru.h>

using namespace realm;

RosGrabberNode::RosGrabberNode()
  : _do_imu_passthrough(false),
    _nrof_frames_received(0),
    _fps(1.0),
    _cam(nullptr),
    _sync_topics(ApproxTimePolicy(5), _sub_input_image, _sub_input_gnss)
{
  readParameters();
  setPaths();

  if (io::fileExists(_file_settings_camera))
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_file_settings_camera, &_fps));
  else
    throw(std::invalid_argument("Error loading camera file: Provided path does not exist."));
  _cam_msg = to_ros::pinhole(_cam);

  ROS_INFO_STREAM(
          "ROS Grabber Node successfully loaded camera: "
                  << "\n\tfps = " << _fps
                  << "\n\tcx = " << _cam->cx()
                  << "\n\tcy = " << _cam->cy()
                  << "\n\tfx = " << _cam->fx()
                  << "\n\tfy = " << _cam->fy()
                  << "\n\tk1 = " << _cam->k1()
                  << "\n\tk2 = " << _cam->k2()
                  << "\n\tp1 = " << _cam->p1()
                  << "\n\tp2 = " << _cam->p2()
                  << "\n\tk3 = " << _cam->k3());

  _sub_heading = _nh.subscribe(_topic_heading, 10, &RosGrabberNode::subHeading, this, ros::TransportHints());
  _sub_relative_altitude = _nh.subscribe(_topic_relative_altitude, 10, &RosGrabberNode::subRelativeAltitude, this, ros::TransportHints());
  _sub_orientation = _nh.subscribe(_topic_orientation, 10, &RosGrabberNode::subOrientation, this, ros::TransportHints());
  _sub_input_image.subscribe(_nh, _topic_image, 10);
  _sub_input_gnss.subscribe(_nh, _topic_gnss, 10);
  _sync_topics.registerCallback(boost::bind(&RosGrabberNode::subImageGnss, this, _1, _2));

  _pub_frame = _nh.advertise<realm_msgs::Frame>(_topic_out_frame, 100);

  if (_do_imu_passthrough)
    _pub_imu = _nh.advertise<sensor_msgs::Imu>(_topic_out_imu, 100);

  ROS_INFO_STREAM("ROS Grabber Node subscribed to topics:\n"
                  "- Image topic:\t\t"  << _topic_image   << "\n"
                  "- GNSS topic:\t\t"   << _topic_gnss    << "\n"
                  "- Heading topic:\t"  << _topic_heading << "\n"
                  "- Rel. Altitude topic:  "  << _topic_relative_altitude << "\n"
                  "- Orientation topic:\t"  << _topic_orientation << "\n"
                  "- Output topic IMU:\t"  << _topic_out_imu << "\n"
                  "- Output topic Frame:\t\t" << _topic_out_frame     << "\n");

}

void RosGrabberNode::readParameters()
{
  ros::NodeHandle param_nh("~");

  param_nh.param("config/id",                    _id_node,                 std::string("uninitialised"));
  param_nh.param("config/profile",               _profile,                 std::string("uninitialised"));
  param_nh.param("config/opt/working_directory", _path_working_directory,  std::string("uninitialised"));
  param_nh.param("topic/image",                  _topic_image,             std::string("uninitialised"));
  param_nh.param("topic/gnss",                   _topic_gnss,              std::string("uninitialised"));
  param_nh.param("topic/heading",                _topic_heading,           std::string("uninitialised"));
  param_nh.param("topic/relative_altitude",      _topic_relative_altitude, std::string("uninitialised"));
  param_nh.param("topic/orientation",            _topic_orientation,       std::string("uninitialised"));
  param_nh.param("topic/out/frame",              _topic_out_frame,         std::string("uninitialised"));
  param_nh.param("topic/out/imu",                _topic_out_imu,           std::string("uninitialised"));

  if (_topic_out_imu != "uninitialised")
    _do_imu_passthrough = true;

  if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
    throw(std::invalid_argument("Error: Working directory does not exist!"));
}

void RosGrabberNode::setPaths()
{
  if (_path_working_directory == "uninitialised")
    _path_working_directory = ros::package::getPath("realm_ros");
  _path_profile = _path_working_directory + "/profiles/" + _profile;

  _file_settings_camera = _path_profile + "/camera/calib.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::invalid_argument("Error: Config folder path '" + _path_profile + "' does not exist!"));
}

void RosGrabberNode::spin()
{
  ros::Rate rate(10*_fps);
  rate.sleep();
}

bool RosGrabberNode::isOkay()
{
  return _nh.ok();
}

void RosGrabberNode::subHeading(const std_msgs::Float64 &msg)
{
  _mutex_heading.lock();
  _heading = msg.data;
  _mutex_heading.unlock();
}

void RosGrabberNode::subRelativeAltitude(const std_msgs::Float64 &msg)
{
  _mutex_relative_altitude.lock();
  _relative_altitude = msg.data;
  _mutex_relative_altitude.unlock();
}

void RosGrabberNode::subOrientation(const sensor_msgs::Imu &msg)
{
  _mutex_orientation.lock();
  _orientation = to_realm::orientation(msg.orientation);
  _mutex_orientation.unlock();

  if (_do_imu_passthrough)
    _pub_imu.publish(msg);
}

void RosGrabberNode::subImageGnss(const sensor_msgs::ImageConstPtr &msg_img, const sensor_msgs::NavSatFixConstPtr &msg_gnss)
{
  cv::Mat img = to_realm::image(*msg_img);

  _mutex_relative_altitude.lock();
  _mutex_heading.lock();

  WGSPose wgs;
  wgs.latitude = msg_gnss->latitude;
  wgs.longitude = msg_gnss->longitude;
  wgs.altitude = msg_gnss->altitude;
  wgs.heading = _heading;

  UTMPose utm = gis::convertToUTM(wgs);

  _mutex_heading.unlock();
  _mutex_relative_altitude.unlock();

  cv::Mat orientation;
  //if (_topic_orientation == "uninitialised" || _orientation.empty())
    orientation = io::computeOrientationFromHeading(utm.heading);
  //else
  //  orientation = _orientation;

  auto frame = std::make_shared<Frame>(_id_node, _nrof_frames_received, Timer::getCurrentTimeMilliseconds(), img, utm, _cam, orientation);

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "/realm";
  _pub_frame.publish(to_ros::frame(header, frame));

  _nrof_frames_received++;
}
