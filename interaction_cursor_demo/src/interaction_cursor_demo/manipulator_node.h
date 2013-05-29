
#ifndef _CAT_MANIPULATOR_NODE_H_
#define _CAT_MANIPULATOR_NODE_H_

#include <cat_user_entity/abstract_interaction_tool.h>
//#include <cat_user_entity/haptic_interaction_tool.h>
#include <cat_user_entity/hydra_interaction_tool.h>
#include <cat_user_entity/tf_scenegraph_object.h>

#include <eigen3/Eigen/Geometry>


namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class ManipulatorNode: public tf::SceneGraphNode {


public:

  enum DeviceType {
    NONE = 0,
    HAPTIC,
    INTERACTIVE_MARKER,
    HYDRA_LEFT,
    HYDRA_RIGHT
  };

  // Methods only!

  ManipulatorNode(const std::string &frame_id,
               tf::TransformListener *tfl, tf::TransformBroadcaster *tfb, DeviceType device_type);

  virtual ~ManipulatorNode();

  void init();


  // Get button stuff should go here, so user can query it?

  bool isGrabbing()
  {
    if( !tool_ ) return false;
    unsigned int index = button_name_map_["grab"];
    return tool_->getToolButtonState(index);
  }

  virtual void timerUpdate()
  {
    tool_->timerUpdate();
  }


protected:
    // Methods

protected:
    // Members

    DeviceType device_type_;
    something::AbstractInteractionTool *tool_;
    std::map<std::string, unsigned int> button_name_map_;
};

}  // namespace something

#endif
