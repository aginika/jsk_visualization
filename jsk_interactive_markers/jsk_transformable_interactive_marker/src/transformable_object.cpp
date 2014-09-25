#include <jsk_transformable_interactive_marker/transformable_object.h>

using namespace jsk_transformable_interactive_marker;

TransformableObject::TransformableObject(){
  ROS_INFO("Init TransformableObject");
}

std::vector<visualization_msgs::InteractiveMarkerControl> TransformableObject::makeRotateTransFixControl(){
  visualization_msgs::InteractiveMarkerControl control;

  std::vector<visualization_msgs::InteractiveMarkerControl> controls;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  return controls;
};

void TransformableObject::addMarker(visualization_msgs::InteractiveMarker &int_marker, bool always_visible, unsigned int interaction_mode)
{
  visualization_msgs::Marker marker = getVisualizationMsgMarker();
  visualization_msgs::InteractiveMarkerControl marker_control;
  marker_control.always_visible = always_visible;
  marker_control.markers.push_back(marker);
  marker_control.interaction_mode = interaction_mode;
  int_marker.controls.push_back(marker_control);
};

void TransformableObject::addControl(visualization_msgs::InteractiveMarker &int_marker, bool fixed)
{
  if(fixed){
    std::vector<visualization_msgs::InteractiveMarkerControl> rotate_controls = makeRotateTransFixControl();
    int_marker.controls.insert(int_marker.controls.end(), rotate_controls.begin(), rotate_controls.end());
  }
};

visualization_msgs::InteractiveMarker TransformableObject::getInteractiveMarker(){
  visualization_msgs::InteractiveMarker int_marker;

  addMarker(int_marker);
  addControl(int_marker);
  int_marker.header.frame_id = frame_id_;
  int_marker.name = name_;
  int_marker.description = description_;
  int_marker.pose = pose_;
  int_marker.scale = getInteractiveMarkerScale();
  return int_marker;
};
