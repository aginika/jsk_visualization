#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_marker_helpers/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <math.h>
#include <jsk_interactive_marker/MarkerMenu.h>
#include <jsk_interactive_marker/MarkerPose.h>

#include <std_msgs/Int8.h>

class InteractiveMarkerInterface {
 public:
  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int type );

  void pub_marker_menu(std::string marker,int menu);
  
  void pub_marker_menuCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu );

  void pub_marker_menuCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu, int type);

  void IMSizeLargeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void IMSizeMiddleCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void IMSizeSmallCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeMoveModeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeMoveModeCb1( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeMoveModeCb2( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeForceModeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeForceModeCb1( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeForceModeCb2( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  
  void targetPointMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void lookAutomaticallyMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void ConstraintCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void ikmodeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void useTorsoCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void updateHeadGoal( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void updateBase( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  visualization_msgs::InteractiveMarker makeBaseMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed);


  void changeMarkerForceMode( std::string mk_name , int im_mode);

  void initControlMarkers(void);


  void initBodyMarkers(void);

  void initHandler(void);

  void changeMarkerMoveMode( std::string mk_name , int im_mode);

  void changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size);

  void changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size, geometry_msgs::PoseStamped dist_pose);

  void changeMarkerOperationModelMode( std::string mk_name );

  InteractiveMarkerInterface ();

  bool markers_set_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
			jsk_interactive_marker::MarkerSetPose::Response &res );

  bool markers_del_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
			jsk_interactive_marker::MarkerSetPose::Response &res );

  bool set_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
                jsk_interactive_marker::MarkerSetPose::Response &res );

  bool reset_cb ( jsk_interactive_marker::SetPose::Request &req,
                  jsk_interactive_marker::SetPose::Response &res );

  void loadMeshes(XmlRpc::XmlRpcValue v);

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::Publisher pub_;
  ros::Publisher pub_move_;
  ros::ServiceServer serv_reset_;
  ros::ServiceServer serv_set_;  
  ros::ServiceServer serv_markers_set_;
  ros::ServiceServer serv_markers_del_;

  tf::TransformListener tfl_;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler menu_handler1;
  interactive_markers::MenuHandler menu_handler2;
  interactive_markers::MenuHandler menu_handler_force;
  interactive_markers::MenuHandler menu_handler_force1;
  interactive_markers::MenuHandler menu_handler_force2;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle2;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle_ik;

  interactive_markers::MenuHandler menu_head_;
  interactive_markers::MenuHandler::EntryHandle head_target_handle_;
  interactive_markers::MenuHandler::EntryHandle head_auto_look_handle_;

  interactive_markers::MenuHandler menu_base_;

  // parameters
  std::string marker_name;
  std::string server_name;
  std::string base_frame;
  std::string move_base_frame;
  std::string target_frame;
  bool fix_marker;
  interactive_markers::MenuHandler::EntryHandle h_mode_last;
  interactive_markers::MenuHandler::EntryHandle h_mode_last2;
  interactive_markers::MenuHandler::EntryHandle h_mode_last3;

  interactive_markers::MenuHandler::EntryHandle use_torso_menu_;
  interactive_markers::MenuHandler::EntryHandle use_torso_t_menu_;
  interactive_markers::MenuHandler::EntryHandle use_torso_nil_menu_;


  int h_mode_rightarm;
  int h_mode_constrained;
  int h_mode_ikmode;
  int use_arm;

  std::list<visualization_msgs::InteractiveMarker> imlist;
  //interactive_markers::MenuHandler menu_handler;

  struct GripperState{
  GripperState() : on_(false), view_facing_(false), edit_control_(false), torso_frame_(false) {}

    bool on_;
    bool view_facing_;
    bool edit_control_;
    bool torso_frame_;
  };

  struct ControlState{
  ControlState() : posture_r_(false), posture_l_(false), torso_on_(false), head_on_(false),
      projector_on_(false), init_head_goal_(false), base_on_(true) {}

    void print()
    {
      ROS_DEBUG_NAMED("control_state", "gripper: on[%d|%d][%d], edit[%d|%d][%d], torso[%d|%d]",
                      l_gripper_.on_, r_gripper_.on_, dual_grippers_.on_, l_gripper_.edit_control_, r_gripper_.edit_control_, dual_grippers_.edit_control_, l_gripper_.torso_frame_, r_gripper_.torso_frame_);
      ROS_DEBUG_NAMED("control_state", "posture[%d|%d] torso[%d] base[%d] head[%d] projector[%d]",
                      posture_l_, posture_r_, torso_on_, base_on_, head_on_, projector_on_ );
    }

    bool posture_r_;
    bool posture_l_;
    bool torso_on_;
    bool head_on_;
    bool projector_on_;
    bool look_auto_on_;
    bool init_head_goal_;
    bool base_on_;
    bool planar_only_;
    GripperState dual_grippers_;
    GripperState r_gripper_;
    GripperState l_gripper_;
  };


  ControlState control_state_;

  geometry_msgs::PoseStamped head_goal_pose_;

  std::string head_link_frame_;
  std::string head_mesh_;
  
  struct MeshProperty{
    std::string link_name;
    std::string mesh_file;
    geometry_msgs::Quaternion orientation;

  };
  std::vector< MeshProperty > rhand_mesh_, lhand_mesh;

};
