

#include <cat_user_entity/manipulator_node.h>

namespace something{

// Constructor
// Must be defined in the header due to library issues in CHAI3D.
ManipulatorNode::ManipulatorNode(const std::string &frame_id,
             tf::TransformListener *tfl, tf::TransformBroadcaster *tfb, DeviceType device_type)
  : SceneGraphNode(frame_id, tfl, tfb),
    device_type_(device_type),
    tool_(0)
{
    init();
}

ManipulatorNode::~ManipulatorNode()
{
    if(tool_) delete tool_;
}

void ManipulatorNode::init()
{

  switch (device_type_)
  {
  case(HYDRA_RIGHT):
  {
    tool_ = new something::HydraInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_, something::HydraInteractionTool::HYDRA_RIGHT);
    break;
  }
  case(HYDRA_LEFT):
  {
    tool_ = new something::HydraInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_, something::HydraInteractionTool::HYDRA_LEFT);
    break;
  }
  case(HAPTIC):
  {
    //tool_ = new something::HapticInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
    break;
  }
  case(INTERACTIVE_MARKER):
  {
    //tool_ = new something::InteractiveMarkerInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
    break;
  }
  default:
    break;
  } // switch

  if(tool_)
    addChild(tool_);
  else
    ROS_ERROR("Constructing manipulator node with no tool type; this isn't supported!");

    button_name_map_["grab"] = 0;
}




} //namespace something
