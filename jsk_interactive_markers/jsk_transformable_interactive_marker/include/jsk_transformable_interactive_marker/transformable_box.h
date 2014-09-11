#ifndef __TRANSFORMABLE_BOX_H__
#define __TRANSFORMABLE_BOX_H__


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <jsk_transformable_interactive_marker/transformable_object.h>

namespace jsk_transformable_interactive_marker
{
  class TransformableBox: public TransformableObject
  {
  public:
    TransformableBox( float length , float r, float g, float b, float a);

    TransformableBox( float x, float y, float z, float r, float g, float b, float a);

    visualization_msgs::Marker getVisualizationMsgMarker();
    void setXYZ( float x , float y, float z){box_x_=x;box_y_=y;box_z_=z;};
    void setRGBA( float r , float g, float b, float a){box_r_=r;box_g_=g;box_b_=b;box_a_=a;};

  float box_x_;
  float box_y_;
  float box_z_;
  float box_r_;
  float box_g_;
  float box_b_;
  float box_a_;
  };
};

#endif
