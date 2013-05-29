#ifndef _CAT_CAMERA_NODE_H_
#define _CAT_CAMERA_NODE_H_


#include <cat_user_entity/tf_scenegraph_object.h>
#include <eigen3/Eigen/Geometry>


namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class CameraNode: public tf::SceneGraphNode {


public:
  // Methods only!

  CameraNode(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb);

  virtual void init();


protected:
    // Methods

protected:
    // Members


};

}  // namespace something

#endif
