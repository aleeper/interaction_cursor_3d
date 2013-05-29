
#include <cat_user_entity/abstract_interaction_tool.h>

namespace something {

// Constructor
AbstractInteractionTool::AbstractInteractionTool(const std::string &frame_id,
         tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
  : SceneGraphNode(frame_id, tfl, tfb),
    handle_(0),
    last_tool_force_(tf::Vector3(0,0,0)),
    last_tool_torque_(tf::Vector3(0,0,0)),
    attached_(false),
    k_linear_(0),
    k_angular_(0)
{
  init();
}


AbstractInteractionTool::~AbstractInteractionTool()
{
  if(handle_) delete handle_;
}

void AbstractInteractionTool::init()
{
  handle_ = new something::AbstractHandle(transform_.child_frame_id_ + "_handle", tfl_, tfb_);
  addChild(handle_);
  handle_->setVisible(false, true);

  ros::NodeHandle nh;
  std::string side = "";
  if(transform_.child_frame_id_.find("right") != std::string::npos) side = "_right";
  else if(transform_.child_frame_id_.find("left") != std::string::npos) side = "_left";

  std::string base_topic = std::string("interaction_cursor") + side;
  subscribe_cursor_ = nh.subscribe<interaction_cursor_msgs::InteractionCursorFeedback>(base_topic + "/feedback", 10,
                                     boost::bind( &AbstractInteractionTool::receiveInteractionCursorFeedback, this, _1 ) );
  publish_cursor_ = nh.advertise<interaction_cursor_msgs::InteractionCursorUpdate>(base_topic + "/update", 10);
}


void AbstractInteractionTool::timerUpdate()
{
  recordButtonTransitions();

  interaction_cursor_msgs::InteractionCursorUpdate update;

  tf::poseTFToMsg(handle_->getTransform(), update.pose.pose);
  update.pose.header.frame_id = handle_->getTransform().frame_id_;
  update.pose.header.stamp = handle_->getTransform().stamp_;

  // process buttons
  if(button_transition_[button_name_map_["click"]] == HIGH) update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::KEEP_ALIVE;
  if(button_transition_[button_name_map_["click"]] == LOW_TO_HIGH)
  {
    if(attached_frame_id_ != "")
    {
      ROS_INFO("Attaching to frame [%s]", attached_frame_id_.c_str());
      update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::GRAB;
      attached_ = true;
    }
    else
    {
      //ROS_INFO("Tried to grab when the attached_frame_id_ was invalid!");
    }
  }
  if(button_transition_[button_name_map_["click"]] == HIGH_TO_LOW) update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::RELEASE;
  if(button_transition_[button_name_map_["click"]] == LOW)         update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::NONE;

  if(button_transition_[button_name_map_["menu"]] == LOW_TO_HIGH)  update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::QUERY_MENU;

  if(button_transition_[button_name_map_["key_up"]]    == LOW_TO_HIGH) update.key_event = interaction_cursor_msgs::InteractionCursorUpdate::KEY_UP;
  if(button_transition_[button_name_map_["key_down"]]  == LOW_TO_HIGH) update.key_event = interaction_cursor_msgs::InteractionCursorUpdate::KEY_DOWN;
  if(button_transition_[button_name_map_["key_left"]]  == LOW_TO_HIGH) update.key_event = interaction_cursor_msgs::InteractionCursorUpdate::KEY_LEFT;
  if(button_transition_[button_name_map_["key_right"]] == LOW_TO_HIGH) update.key_event = interaction_cursor_msgs::InteractionCursorUpdate::KEY_RIGHT;
  if(button_transition_[button_name_map_["key_enter"]] == LOW_TO_HIGH) update.key_event = interaction_cursor_msgs::InteractionCursorUpdate::KEY_ENTER;
  if(button_transition_[button_name_map_["key_esc"]]   == LOW_TO_HIGH) update.key_event = interaction_cursor_msgs::InteractionCursorUpdate::KEY_ESCAPE;

  //update.key_event =

  updateVirtualCoupling();

  //ROS_DEBUG("At end of timerUpdate, attached_frame_id_ = [%s], attached_ = [%d]", attached_frame_id_.c_str(), attached_);
  publish_cursor_.publish(update);
}


void AbstractInteractionTool::updateVirtualCoupling()
{
  // Skip this if we aren't grabbing anything with an associated control frame...
  if(!attached_ || attached_frame_id_ == "" || attached_frame_id_ == "no_frame")
  {
    //ROS_INFO("It doesn't seem we are attached, so not updating virtual coupling...");
    setToolForce(tf::Vector3(0,0,0));
    setToolTorque(tf::Vector3(0,0,0));
    return;
  }

  // Make sure we are doing all calculations in the tool (e.g. device) frame.
  tf::StampedTransform tool_T_handle_stamped = handle_->getTransform();
  tf::StampedTransform tool_T_attached_stamped;
  tfl_->lookupTransform(getFrameId(), attached_frame_id_, ros::Time(0), tool_T_attached_stamped);

  tf::Transform tool_T_handle = tool_T_handle_stamped;
  tf::Transform tool_T_attached = tool_T_attached_stamped;
  tf::Transform tool_T_grasp = tool_T_attached * attached_frame_T_grasp_;

  // Should be at current location of grasp point, but we measure current location of handle.
  tf::Vector3 position_handle_to_grasp_point = tool_T_grasp.getOrigin() - tool_T_handle.getOrigin();

  // Get rotation from handle frame to grasp frame
  tf::Quaternion quaternion_handle_to_grasp = tool_T_handle.getRotation().inverse()*tool_T_grasp.getRotation();
  tf::Vector3 angle_axis_handle_to_grasp = quaternion_handle_to_grasp.getAngle()*quaternion_handle_to_grasp.getAxis();


  tf::Vector3 force = k_linear_*position_handle_to_grasp_point;
  tf::Vector3 torque = k_angular_*angle_axis_handle_to_grasp;

  setToolForce(force);
  setToolTorque(torque);
}

void AbstractInteractionTool::drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array, int action)
{
  // TODO re-write this to avoid copying lines.

  // add markers for forces and torques
  visualization_msgs::Marker marker;
  marker.header.frame_id = getFrameId();
  marker.header.stamp = now;

  marker.type = marker.ARROW;
  marker.ns = getFrameId();
  marker.scale.x = 0.025;  // shaft radius
  marker.scale.y = 0.05;   // head radius
  marker.scale.z = 0.1;

  // force
  if(true)
  {
    marker.id = 0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.2;
    marker.color.a = 0.9;

    tf::Vector3 tail_position = handle_->getPosition();
    tf::Vector3 tip_position = tail_position + last_tool_force_;
    float length = (tip_position - tail_position).length();

    if( length > 0.00001 && (action == marker.ADD) )
    {
      marker.action = action;
      marker.scale.z = 0.25*length;
      geometry_msgs::Point tail_point, tip_point;
      tf::pointTFToMsg(tail_position, tail_point);
      tf::pointTFToMsg(tip_position, tip_point);

      marker.points.push_back(tail_point);
      marker.points.push_back(tip_point);

    }
    else
    {
      marker.action = marker.DELETE;
    }
  }
  array.markers.push_back(marker);

  // torque
  if(false)
  {
    marker.id = 1;
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.2;
    marker.color.a = 0.9;

    tf::Vector3 tail_position = handle_->getPosition();
    tf::Vector3 tip_position = tail_position + last_tool_torque_;
    float length = (tip_position - tail_position).length();

    if( length > 0.00001 && (action == marker.ADD))
    {
      marker.scale.z = 0.25*length;
      marker.action = action;
      geometry_msgs::Point tail_point, tip_point;
      tf::pointTFToMsg(tail_position, tail_point);
      tf::pointTFToMsg(tip_position, tip_point);

      marker.points.push_back(tail_point);
      marker.points.push_back(tip_point);

    }
    else
    {
      marker.action = marker.DELETE;
    }
  }
  array.markers.push_back(marker);
}

void AbstractInteractionTool::receiveInteractionCursorFeedback(const interaction_cursor_msgs::InteractionCursorFeedbackConstPtr& icf_cptr)
{



  switch(icf_cptr->event_type)
  {
  case interaction_cursor_msgs::InteractionCursorFeedback::NONE:
  case interaction_cursor_msgs::InteractionCursorFeedback::GRABBED:
    //ROS_INFO_STREAM("Received feedback of type [" << (int)icf_cptr->event_type << "] with pose " << icf_cptr->pose);
    attached_frame_id_ = icf_cptr->pose.header.frame_id;
    tf::poseMsgToTF(icf_cptr->pose.pose, attached_frame_T_grasp_);
    attachment_type_ = icf_cptr->attachment_type;
    break;
  case interaction_cursor_msgs::InteractionCursorFeedback::KEEP_ALIVE:
    // Do nothing?
    break;
  case interaction_cursor_msgs::InteractionCursorFeedback::RELEASED:
      ROS_INFO("Received RELEASED feedback!");
      attached_ = false;
      break;
  case interaction_cursor_msgs::InteractionCursorFeedback::LOST_GRASP:
    ROS_WARN("Received LOST_GRASP feedback!");
    attached_ = false;
    break;
  default:
    break;
  }
}

void AbstractInteractionTool::recordButtonTransitions()
{
  if(button_state_.size() != button_transition_.size())
  {
    ROS_ERROR("Button state and button transition vector are not same size, aborting!");
    return;
  }

  for(size_t i = 0; i < button_state_.size(); i++)
  {
    bool now = button_state_[i];
    bool previous = (button_transition_[i] == HIGH) || (button_transition_[i] == LOW_TO_HIGH);
    if     (  now && !previous )  button_transition_[i] = LOW_TO_HIGH;
    else if(  now &&  previous )  button_transition_[i] = HIGH;
    else if( !now &&  previous )  button_transition_[i] = HIGH_TO_LOW;
    else if( !now && !previous )  button_transition_[i] = LOW;
    //if(i == 0)  ROS_INFO("now = %d, previous = %d, transition = %d", now, previous, button_transition_[i]);
  }
}


} // namespace
