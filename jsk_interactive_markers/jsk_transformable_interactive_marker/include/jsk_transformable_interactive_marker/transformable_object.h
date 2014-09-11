#ifndef __TRANSFORMABLE_OBJECT_H__
#define __TRANSFORMABLE_OBJECT_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <vector>

namespace jsk_transformable_interactive_marker {
  class TransformableObject{
  public:
    TransformableObject();

    std::vector<visualization_msgs::InteractiveMarkerControl> makeRotateTransFixControl();

    visualization_msgs::InteractiveMarker getInteractiveMarker();
    virtual visualization_msgs::Marker getVisualizationMsgMarker(){};
    void addMarker(visualization_msgs::InteractiveMarker &int_marker, bool always_visible = true, unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D);
    void addControl(visualization_msgs::InteractiveMarker &int_marker, bool fixed = true);

    visualization_msgs::Marker marker_;
  };

};

#endif
