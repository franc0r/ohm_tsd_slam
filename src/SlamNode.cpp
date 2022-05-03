/*
 * SlamNode.cpp
 *
 *  Created on: 05.05.2014
 *      Author: phil
 * Refactored by: Christian Wendt
 */

#include "SlamNode.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"

#include "obcore/math/mathbase.h"
#include <functional>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace ohm_tsd_slam
{
SlamNode::SlamNode()
  : Node("slam_node")
{
  int iVar                   = 0;
  double gridPublishInterval = 0.0;
  double loopRateVar         = 0.0;
  double truncationRadius    = 0.0;
  double cellSize            = 0.0;
  unsigned int octaveFactor  = 0;
  double xOffset = 0.0;
  double yOffset = 0.0;
  std::string topicLaser;
  std::string topicServiceStartStop;

  this->declare_parameter<int>("robot_nbr", 1);
  this->declare_parameter<double>("x_off_factor", 0.5);
  this->declare_parameter<double>("y_off_factor", 0.5);
  this->declare_parameter<double>("x_offset", 0.0);
  this->declare_parameter<double>("y_offset", 0.0);

  iVar = this->get_parameter("robot_nbr").as_int();
  unsigned int robotNbr = static_cast<unsigned int>(iVar);
  _xOffFactor = this->get_parameter("x_off_factor").as_double();
  _yOffFactor = this->get_parameter("y_off_factor").as_double();
  xOffset = this->get_parameter("x_offset").as_double();
  yOffset = this->get_parameter("y_offset").as_double();

  this->declare_parameter<int>("map_size", 10);
  this->declare_parameter<double>("cellsize", 0.025);
  this->declare_parameter<int>("truncation_radius", 3);
  this->declare_parameter<double>("occ_grid_time_interval", 2.0);
  this->declare_parameter<double>("loop_rate", 40.0);
  this->declare_parameter<std::string>("laser_topic", "scan");
  this->declare_parameter<std::string>("topic_service_start_stop", "start_stop_slam");

  iVar = this->get_parameter("map_size").as_int();
  octaveFactor = static_cast<unsigned int>(iVar);
  cellSize = this->get_parameter("cellsize").as_double();
  iVar = this->get_parameter("truncation_radius").as_int();
  truncationRadius = static_cast<double>(iVar);
  gridPublishInterval = this->get_parameter("occ_grid_time_interval").as_double();
  loopRateVar = this->get_parameter("loop_rate").as_double();
  topicLaser = this->get_parameter("laser_topic").as_string();
  topicServiceStartStop = this->get_parameter("topic_service_start_stop").as_string();

  _loopRate = std::make_unique<rclcpp::Rate>(loopRateVar);
  _gridInterval = std::make_unique<rclcpp::Duration>(rclcpp::Duration::from_seconds(gridPublishInterval));

  if(octaveFactor > 15)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error! Unknown map size -> set to default!");
    octaveFactor = 10;
  }
  //instanciate representation
  _grid = new obvious::TsdGrid(cellSize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(octaveFactor));  //obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(truncationRadius * cellSize);
  unsigned int cellsPerSide = pow(2, octaveFactor);
  double sideLength = static_cast<double>(cellsPerSide) * cellSize;
  RCLCPP_INFO_STREAM(this->get_logger(), "Creating representation with " << cellsPerSide << "x" << cellsPerSide << "cells, representating " <<
                     sideLength << "x" << sideLength << "m^2");

  //instanciate mapping threads
  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid, this->shared_from_this(), xOffset, yOffset);

  ThreadLocalize* threadLocalize = nullptr;
  TaggedSubscriber subs;
  std::string nameSpace = "";

  //instanciate localization threads
  if(robotNbr == 1)  //single slam
  {
    threadLocalize = new ThreadLocalize(_grid, _threadMapping, this->shared_from_this(), nameSpace, xOffset, yOffset);
    subs = TaggedSubscriber(topicLaser, *threadLocalize, shared_from_this());
    subs.switchOn();
    //subs = _nh.subscribe(topicLaser, 1, &ThreadLocalize::laserCallBack, threadLocalize);
    _subsLaser.push_back(subs);
    _localizers.push_back(threadLocalize);
    RCLCPP_INFO_STREAM(this->get_logger(), "Single SLAM started");
  }
  else
  {
    for(unsigned int i = 0; i < robotNbr; i++)   //multi slam
    {
      std::stringstream sstream;
      sstream << "robot";
      sstream << i << "/namespace";
      std::string dummy = sstream.str();
      this->declare_parameter<std::string>(dummy, "default_ns");
      nameSpace = this->get_parameter(dummy).as_string();

      threadLocalize = new ThreadLocalize(_grid, _threadMapping, this->shared_from_this(), nameSpace, xOffset, yOffset);
//      subs = _nh.subscribe(nameSpace + "/" + topicLaser, 1, &ThreadLocalize::laserCallBack, threadLocalize);
      subs = TaggedSubscriber(nameSpace + "/" + topicLaser, *threadLocalize, this->shared_from_this());
      _subsLaser.push_back(subs);
      _localizers.push_back(threadLocalize);
      RCLCPP_INFO_STREAM(this->get_logger(), "started for thread for " << nameSpace);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Multi SLAM started!");
  }
  _serviceStartStopSLAM = this->create_service<ohm_tsd_slam::srv::StartStopSLAM>(
    topicServiceStartStop,
    std::bind(&SlamNode::callBackServiceStartStopSLAM, this, std::placeholders::_1, std::placeholders::_2)
  );
  // rclcpp::create_timer(*this, this->get_clock(), _loopRate, std::bind(&SlamNode::timedGridPub, this));
}

SlamNode::~SlamNode()
{
  //stop all localization threads
  for(std::vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }

  //stop mapping threads
  _threadGrid->terminateThread();
  while(_threadGrid->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadGrid;
  _threadMapping->terminateThread();
  while(_threadMapping->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadMapping;
  delete _grid;
}

void SlamNode::timedGridPub()
{
  static rclcpp::Time lastMap = this->get_clock()->now();
  const rclcpp::Time curTime = this->get_clock()->now();

  if((curTime - lastMap) > *_gridInterval)
  {
    _threadGrid->unblock();
    lastMap = curTime;
  }
}

// void SlamNode::run()
// {
//   RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for first laser scan to initialize node...");
//   ROS_INFO_STREAM("Waiting for first laser scan to initialize node...\n");
//   while(ros::ok())
//   {
//     ros::spinOnce();
//     this->timedGridPub();
//     _loopRate->sleep();
//   }
// }

bool SlamNode::callBackServiceStartStopSLAM(const std::shared_ptr<ohm_tsd_slam::srv::StartStopSLAM::Request> req,
                                            std::shared_ptr<ohm_tsd_slam::srv::StartStopSLAM::Response>)
{
  TaggedSubscriber* subsCur = NULL;
  for(auto iter = _subsLaser.begin(); iter < _subsLaser.end(); iter++)
  {
    if(iter->topic(req->topic))
      subsCur = &*iter;
  }
  if(!subsCur)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), __PRETTY_FUNCTION__ << " Error! Topic " << req->topic << " invalid!");
    return false;
  }
  if(req->start_stop == req->START)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), __PRETTY_FUNCTION__ << " Started SLAM for topic " << req->topic);
    subsCur->switchOn();
  }
  else if(req->start_stop == req->STOP)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), __PRETTY_FUNCTION__ << " Stopped SLAM for topic " << req->topic);
    subsCur->switchOff();
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), __PRETTY_FUNCTION__ << " Error. Unknown request for service");
    return false;
  }
  return true;
}

} /* namespace ohm_tsd_slam */
