#ifndef __TRANSFORMABLE_INTERACTIVE_SERVER_H__
#define __TRANSFORMABLE_INTERACTIVE_SERVER_H__

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_transformable_interactive_marker/transformable_box.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <jsk_rviz_plugins/OverlayText.h>

using namespace std;

namespace jsk_transformable_interactive_marker
{
  class TransformableInteractiveServer{
  public:
    TransformableInteractiveServer();

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void resizeBox(std_msgs::Float32 msg);
    void setPose(geometry_msgs::PoseStamped msg);
    void addPoseDiff(geometry_msgs::PoseStamped msg);
    void insert(std::string name);
    void erase(std::string name);

    void run();
    void focusTextPublish();

    std::string focus_object_marker_name_;
    ros::NodeHandle* n_;
    ros::Subscriber resize_sub_;
    ros::Subscriber setpose_sub_;
    ros::Subscriber addposediff_sub_;
    ros::Publisher focus_text_pub_;
    interactive_markers::InteractiveMarkerServer* server_;
    map<string, TransformableObject*> mapstriTable; 
  };
}

#endif
