#ifndef _CAT_ABSTRACT_HANDLE_H_
#define _CAT_ABSTRACT_HANDLE_H_

#include <cat_user_entity/tf_scenegraph_object.h>
#include <eigen3/Eigen/Geometry>

namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class AbstractHandle: public tf::SceneGraphNode{

public:
// Methods only!

  // Constructor
  AbstractHandle(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb);

  virtual ~AbstractHandle();

  virtual void init();

  virtual void drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array, int action);


protected:
// Methods


// Members

    //something::AbstractHandle *handle_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

};

}  // namespace something

#endif
