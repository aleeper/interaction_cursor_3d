#ifndef _CAT_ABSTRACT_INTERACTION_TOOL_H_
#define _CAT_ABSTRACT_INTERACTION_TOOL_H_


#include <cat_user_entity/tf_scenegraph_object.h>
#include <cat_user_entity/abstract_handle.h>
#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include <interaction_cursor_msgs/InteractionCursorFeedback.h>


#include <eigen3/Eigen/Geometry>

namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class AbstractInteractionTool: public tf::SceneGraphNode{

public:
  // Methods only!

  AbstractInteractionTool(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb);

  virtual ~AbstractInteractionTool();

  virtual void init();

  const something::AbstractHandle* getHandle()
  {
      return handle_;
  }


  // Read the state of the binary switches on the tool.
  bool getToolButtonState(const unsigned int &index) const
  {
    if(index >= getToolButtonCount()) return false;
    //if(attached_) ROS_INFO("Currently attached, lieing to parent.");
    return button_state_[index] && !attached_;
  }

  // Get the number of buttons available on the tool.
  unsigned int getToolButtonCount() const
  {
    return (unsigned int)button_state_.size();
  }

  // Set the force applied to the tool
  virtual void setToolForce(const Vector3 &force)           { last_tool_force_ = force; }

  // Set the torque applied to the tool
  virtual void setToolTorque(const Vector3 &torque)         { last_tool_torque_ = torque; }

  // Set the force and torque applied to the tool
  virtual void setToolForceAndTorque(const Vector3 &force, const Vector3 &torque) { setToolForce(force); setToolTorque(torque); }

//    // Set the gripper force on the tool
//    virtual void setToolGripperForce(const float &force);

  virtual void attachHandleToTfFrame(const std::string& attached_frame_id, const tf::Pose& attached_frame_pose)
  {
    attached_frame_id_ = attached_frame_id;
    attached_frame_T_grasp_ = attached_frame_pose;
  }

    virtual void timerUpdate();


protected: 
// Methods

  virtual void receiveInteractionCursorFeedback(const interaction_cursor_msgs::InteractionCursorFeedbackConstPtr& icf_cptr);

  // Used to initialize the button storage
  virtual void setToolButtonCount(const unsigned int &count)
  {
    ROS_INFO("Setting tool button count to %d", count);
    button_state_.resize(count, false);
    button_transition_.resize(count, LOW);
  }

  virtual void setToolButtonState(const size_t &index, const bool &state)
  {
    if(index >= button_state_.size())
    {
      ROS_ERROR("Can't set button %zd state, max size is %zd", index, button_state_.size());
      return;
    }
    button_state_[index] = state;
  }

  virtual void updateVirtualCoupling();

  virtual void drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array, int action);

  virtual void recordButtonTransitions();

//    virtual void updateDevice()
//    {
//      // Here is where, for example, we:
//      // readDevicePosition();
//      // getAttachedFramePosition();
//      // computeVirtualCouplingForce();
//      // sendDeviceForce();
//    }


// Members

  ros::Publisher  publish_cursor_;
  ros::Subscriber subscribe_cursor_;

  something::AbstractHandle *handle_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

  std::string attached_frame_id_;
  tf::Pose attached_frame_T_grasp_;
  bool attached_;
  uint8_t attachment_type_;
//  enum AttachmentType{
//    NONE = interaction_cursor_msgs::InteractionCursorFeedback::NONE,
//    POSITION = interaction_cursor_msgs::InteractionCursorFeedback::POSITION,
//    ORIENTATION = interaction_cursor_msgs::InteractionCursorFeedback::ORIENTATION,
//    POSITION_AND_ORIENTATION = interaction_cursor_msgs::InteractionCursorFeedback::POSITION_AND_ORIENTATION
//  } attachment_type_;

  enum buttonTransition {
    LOW = 0,
    HIGH = 1,
    HIGH_TO_LOW = 2,
    LOW_TO_HIGH = 3
  };

  float k_linear_;
  float k_angular_;
  std::vector<bool> button_state_;
  std::vector<buttonTransition> button_transition_;
  std::map<std::string, unsigned int> button_name_map_;


};

}  // namespace something

#endif
