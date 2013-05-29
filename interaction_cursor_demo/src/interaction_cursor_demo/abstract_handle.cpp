
#include <cat_user_entity/abstract_handle.h>


namespace something {

// Constructor
AbstractHandle::AbstractHandle(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
  : SceneGraphNode(frame_id, tfl, tfb)
{
  init();
}

AbstractHandle::~AbstractHandle()
{

}

void AbstractHandle::init()
{
//  handle_ = new something::AbstractHandle(transform_.child_frame_id_ + "_handle", tfl_, tfb_);
//  addChild(handle_);
}


void AbstractHandle::drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array, int action)
{
  // add markers handle
  visualization_msgs::Marker marker;
  marker.action = action;
  marker.header.frame_id = getFrameId();
  marker.header.stamp = now;
  // Transform is identity because we are in this frame!
  marker.pose.orientation.w = 1;

  marker.ns = getFrameId();
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.2;
  marker.color.a = 0.7;


  marker.type = marker.ARROW;
  marker.scale.x = 0.2;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;

  array.markers.push_back(marker);
}


}  // namespace
