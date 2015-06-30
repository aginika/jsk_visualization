// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

#include <ros/ros.h>
#include <rviz/tool_manager.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>
#include "record_rosbag_tool.h"


namespace jsk_rviz_plugins
{
  RecordRosbagTool::RecordRosbagTool()
    : rviz::Tool()
  {
    recording = false;
  }

  RecordRosbagTool::~RecordRosbagTool()
  {
  }

  void RecordRosbagTool::onInitialize()
  {

  }

  void RecordRosbagTool::openProperty(
    rviz::Property* property)
  {
    property->expand();
    if (property->numChildren() > 0) {
      for (size_t i = 0; i < property->numChildren(); i++) {
        openProperty(property->childAt(i));
      }
      context_->queueRender();
    }
  }
  
  void RecordRosbagTool::activate()
  {
    if(!recording){
      recording = true;
      std::stringstream ss;
      ss << "Activate Rosbag Recording... ";
      setStatus( QString( ss.str().c_str() ) );    

      ros::V_string topics;
      ros::this_node::getSubscribedTopics(topics);

      ROS_INFO("Start Subscribing Below Topics");
      rosbag::RecorderOptions rco;
      for (unsigned int i = 0; i < topics.size(); ++i)
      	{
      	  rco.topics.push_back(topics.at(i));
      	  ROS_INFO((std::string("\t")+topics.at(i)).c_str());
      	}

      recorder = new rosbag::Recorder(rco);
      recorder->run();
    }
  }

  void RecordRosbagTool::deactivate()
  {
    std::stringstream ss;
    ss << "Deactivate";
    setStatus( QString( ss.str().c_str() ) );    
    ROS_ERROR(ss.str().c_str());
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::RecordRosbagTool, rviz::Tool )
