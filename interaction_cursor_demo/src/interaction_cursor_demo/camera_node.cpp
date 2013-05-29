

#include <cat_user_entity/camera_node.h>

namespace something {

// Constructor
CameraNode::CameraNode(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
  : SceneGraphNode(frame_id, tfl, tfb)
{
  init();
}

void CameraNode::init()
{

}






} // namespace
