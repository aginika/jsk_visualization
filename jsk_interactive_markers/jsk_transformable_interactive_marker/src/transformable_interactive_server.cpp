#include <jsk_transformable_interactive_marker/transformable_interactive_server.h>

using namespace jsk_transformable_interactive_marker;

TransformableInteractiveServer::TransformableInteractiveServer():n_(new ros::NodeHandle){
  resize_sub_ = n_->subscribe("resize", 1, &TransformableInteractiveServer::resizeBox, this);
  setpose_sub_ = n_->subscribe("set_pose", 1, &TransformableInteractiveServer::setPose, this);
  addposediff_sub_ = n_->subscribe("add_pose_diff", 1, &TransformableInteractiveServer::addPoseDiff, this);
  focus_text_pub_ = n_->advertise<jsk_rviz_plugins::OverlayText>("focus_marker_name", 1);

  server_ = new interactive_markers::InteractiveMarkerServer("simple_marker");

  TransformableBox transformable_box(0.45, 0.45, 0.45, 0.5, 0.5, 0.5, 1.0);
  visualization_msgs::InteractiveMarker int_marker = transformable_box.getInteractiveMarker();
  int_marker.header.frame_id = "/map";
  int_marker.name = "my_marker";
  int_marker.description = int_marker.name;

  server_->insert(int_marker, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));

  visualization_msgs::InteractiveMarker int_marker2 = transformable_box.getInteractiveMarker();
  int_marker2.header.frame_id = "/map";
  int_marker2.name = "my_marker2";
  int_marker2.description = int_marker2.name;

  server_->insert(int_marker2, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));

  server_->applyChanges();
}

void TransformableInteractiveServer::processFeedback(
                                                     const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM("Button Click Focus on :" << feedback->marker_name);
      focus_object_marker_name_ = feedback->marker_name;
      focusTextPublish();
      break;
    }
}

void TransformableInteractiveServer::resizeBox(std_msgs::Float32 msg){
  visualization_msgs::InteractiveMarker tmp_marker;
  server_->get("my_marker", tmp_marker);
  tmp_marker.controls[0].markers[0].scale.x = msg.data;
  server_->insert(tmp_marker);
  server_->applyChanges();
};

void TransformableInteractiveServer::setPose(geometry_msgs::PoseStamped msg){
  server_->setPose(focus_object_marker_name_, msg.pose, msg.header);
  server_->applyChanges();
}

void TransformableInteractiveServer::addPoseDiff(geometry_msgs::PoseStamped msg){
  visualization_msgs::InteractiveMarker int_marker;
  server_->get(focus_object_marker_name_, int_marker);
  int_marker.pose.position.x += msg.pose.position.x;
  int_marker.pose.position.y += msg.pose.position.y;
  int_marker.pose.position.z += msg.pose.position.z;
  int_marker.pose.orientation.x += msg.pose.orientation.x;
  int_marker.pose.orientation.y += msg.pose.orientation.y;
  int_marker.pose.orientation.z += msg.pose.orientation.z;
  int_marker.pose.orientation.w += msg.pose.orientation.w;
  server_->setPose(focus_object_marker_name_, int_marker.pose, msg.header);
  server_->applyChanges();
}

void TransformableInteractiveServer::insert(std::string name){
}

void TransformableInteractiveServer::erase(std::string name){
}

void TransformableInteractiveServer::focusTextPublish(){
  jsk_rviz_plugins::OverlayText focus_text;
  focus_text.text = focus_object_marker_name_;
  focus_text.top = 0;
  focus_text.left = 0;
  focus_text.width = 300;
  focus_text.height = 50;
  focus_text.bg_color.r = 0.9;
  focus_text.bg_color.b = 0.9;
  focus_text.bg_color.g = 0.9;
  focus_text.bg_color.a = 0.1;
  focus_text.fg_color.r = 0.3;
  focus_text.fg_color.g = 0.3;
  focus_text.fg_color.b = 0.8;
  focus_text.fg_color.a = 1;
  focus_text.line_width = 1;
  focus_text.text_size = 30;
  focus_text_pub_.publish(focus_text);
}

void TransformableInteractiveServer::run(){
  ros::spin();
}
