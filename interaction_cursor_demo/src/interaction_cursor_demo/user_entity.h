#ifndef _CAT_USER_ENTITY_H_
#define _CAT_USER_ENTITY_H_

#include <cat_user_entity/tf_scenegraph_object.h>
#include <cat_user_entity/camera_node.h>
#include <cat_user_entity/manipulator_node.h>

#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp>

namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class UserEntity: public tf::SceneGraphNode {


public:

  // Methods only!

  // Constructor
  UserEntity(const std::string& tf_parent_frame_id, const std::string& frame_prefix,
             tf::TransformListener* tfl, tf::TransformBroadcaster* tfb, ros::Publisher* pub_markers);

  virtual ~UserEntity();

  void init(const std::string &device);

  void attachCoupling();

  void update();

  void changeParentFrameId(const std::string &parent_id);

  bool getGrabState();

  void updateClutch();

protected:
    // Methods

protected:
    // Members

    // The transform from workspace to handle when we started moving the world
    Transform clutch_start_transform_;

    tf::StampedTransform grab_start_world_to_handle_;

    // Is there any good reason to support N widgets?
    something::ManipulatorNode *right_, *left_;
    something::CameraNode *view_;

    std::string prefix_;

    ros::Timer update_timer_;

    bool grabbing_;

    float user_workspace_separation_;
};

}  // namespace something

#endif
