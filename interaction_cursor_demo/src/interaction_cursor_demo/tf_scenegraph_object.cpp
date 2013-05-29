

#include <cat_user_entity/tf_scenegraph_object.h>
#include <visualization_msgs/MarkerArray.h>

namespace tf {

SceneGraphNode::SceneGraphNode(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb, ros::Publisher* pub_markers)
  : tfl_(tfl), tfb_(tfb), pub_markers_(pub_markers), parent_(0), visible_(true)
{
    transform_.child_frame_id_ = frame_id;
    transform_.setIdentity();
}

SceneGraphNode::~SceneGraphNode()
{
    // cleanup...
}

void SceneGraphNode::setPosition(const tf::Vector3 &position)   { transform_.setOrigin(position); }
void SceneGraphNode::setQuaternion(const tf::Quaternion &quaternion)   { transform_.setRotation(quaternion); }

void SceneGraphNode::setTransform(const tf::Transform &transform)
{
    transform_.setOrigin(transform.getOrigin());
    transform_.setRotation(transform.getRotation());
}

tf::Vector3           SceneGraphNode::getPosition() const   { return transform_.getOrigin();   }
tf::Quaternion        SceneGraphNode::getQuaternion() const { return transform_.getRotation(); }
const tf::StampedTransform&  SceneGraphNode::getTransform() const  { return transform_; }


tf::SceneGraphNode* SceneGraphNode::accessChild(const std::string &key)
{
    tf::SceneGraphNode* node = 0;
    if(getFrameId() == key) return this;

    // Recursively check children
    std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
    for( ; it != children_.end(); it++)
    {
        node = it->second->accessChild(key);
        if(node) return node;
    }
    return 0;
}

void SceneGraphNode::addChild(tf::SceneGraphNode *node)
{
    node->setParent(this);
    children_[node->getFrameId()] = node;
}

bool SceneGraphNode::removeChild(tf::SceneGraphNode *node)
{
    std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
    for( ; it != children_.end(); it++)
    {
        if(it->second == node)
        {
            children_.erase(it);
            return true;
        }
    }
    return false;
}

bool SceneGraphNode::removeChild(const std::string &key)
{
    return (bool)children_.erase(key); // returns 1 if it erased a child, 0 otherwise
}

void SceneGraphNode::printChildren(const bool &recursive)
{
    std::vector<std::string> names;
    names.reserve(children_.size());

    // Get children
    std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
    for( ; it != children_.end(); it++)
    {
        names.push_back(it->first);
    }


    printf("Frame %s has %zd children: ", getFrameId().c_str(), names.size());
    std::string children_string = "";
    for(size_t i = 0; i < names.size(); i++)
    {
        children_string += names[i] + " ";
    }
    printf("%s\n", children_string.c_str());

    if(recursive)
    {
        // Recurse down
        std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
        for( ; it != children_.end(); it++)
        {
            it->second->printChildren(recursive);
        }
    }
}

void SceneGraphNode::setVisible(bool visible, bool recurse)
{
  visible_ = visible;

  if(recurse)  // Recursively set children
  {
    std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
    for( ; it != children_.end(); it++)
    {
      it->second->setVisible(visible, recurse);
    }
  }
}


std::string SceneGraphNode::getFrameId()
{
    return transform_.child_frame_id_;
}

std::string SceneGraphNode::getParentFrameId()
{
    return transform_.frame_id_;
}


void SceneGraphNode::publishTransformTree(const ros::Time now)
{
    std::vector<tf::StampedTransform> transforms;
    addTransformsToVector(now, transforms);
    tfb_->sendTransform(transforms);
}

void SceneGraphNode::addTransformsToVector(const ros::Time now, std::vector<tf::StampedTransform>& transforms)
{
    // Add this node to the list
    transform_.stamp_ = now;
    transforms.push_back(transform_);

    // Recursively add all children to the list
    std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
    for( ; it != children_.end(); it++)
    {
        it->second->addTransformsToVector(now, transforms);
    }
}

// Default implementation does nothing!
void SceneGraphNode::drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array, int action)
{

}

void SceneGraphNode::addMarkersToArray(const ros::Time now, visualization_msgs::MarkerArray& array)
{
  // Add the markers for this node to the array
  int action = visible_ ? (visualization_msgs::Marker::ADD) : (visualization_msgs::Marker::DELETE);
  drawSelf(now, array, action);

  // Recursively go through children
  std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
  for( ; it != children_.end(); it++)
  {
    it->second->addMarkersToArray(now, array);
  }
}

void SceneGraphNode::publishMarkers( const bool &recursive)
{
  if(!pub_markers_) return;

  visualization_msgs::MarkerArray array;
  ros::Time now = ros::Time::now();

  if(recursive)
  {
    addMarkersToArray(now, array);
  }
  else
  {
    int action = visible_ ? (visualization_msgs::Marker::ADD) : (visualization_msgs::Marker::DELETE);
    drawSelf(now, array, action);
  }

  pub_markers_->publish(array);

}

/////////////////////////////////////////////////////////////////////
// PROTECTED FUNCTIONS LIVE UNDER HERE
/////////////////////////////////////////////////////////////////////

void SceneGraphNode::setParent(tf::SceneGraphNode* const parent)
{
    if(parent_)
        parent_->removeChild(getFrameId());
    parent_ = parent;
    setParentFrameId(parent->getFrameId());
}

void SceneGraphNode::setParentFrameId(const std::string &parent_id)
{
    transform_.frame_id_ = parent_id;
}


} // namespace tf
