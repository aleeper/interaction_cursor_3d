
#include <cat_user_entity/hydra_interaction_tool.h>
#include <razer_hydra/Hydra.h>

#include <tf/tf.h>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>



namespace something {


HydraInteractionTool::HydraInteractionTool(const std::string &frame_id, tf::TransformListener *tfl,
                     tf::TransformBroadcaster *tfb, PaddleSide side)
: AbstractInteractionTool(frame_id, tfl, tfb),
  workspace_radius_(0.5),
  paddle_side_(side),
  paddle_index_(0)
{
  // Finish other intialization stuff.
  init();
}

HydraInteractionTool::~HydraInteractionTool()
{
}


void HydraInteractionTool::init()
{

//    setPosition(tf::Vector3(pos.x(), pos.y(), pos.z()));
//    setQuaternion(tf::createQuaternionFromYaw(M_PI)*tf::createQuaternionFromRPY(0, 0.4, 0));

  ros::NodeHandle nh;
  hydra_sub_ = nh.subscribe<razer_hydra::Hydra>("hydra_calib", 1, boost::bind(&HydraInteractionTool::updateFromMsg, this, _1));

  // TODO this should come from a config file!
  //button_name_map_["click"] = 7 + 0;

  button_name_map_["key_enter"] = 3;
  button_name_map_["key_esc"]   = 1;
  button_name_map_["menu"]      = 6;
  button_name_map_["click"]     = 7 + 0;
  button_name_map_["key_right"] = 7 + 1;
  button_name_map_["key_left"]  = 7 + 2;
  button_name_map_["key_up"]    = 7 + 3;
  button_name_map_["key_down"]  = 7 + 4;

  setToolButtonCount(7 + 4 + 1);

  k_linear_ = 1;
  k_angular_ = 1;

  ros::NodeHandle pnh("~");
  pnh.param<double>("hydra_workspace_radius", workspace_radius_, 1.0);

  updatePaddleIndex();
}

void HydraInteractionTool::setPaddleSide(HydraInteractionTool::PaddleSide side)
{
  paddle_side_ = side;
  updatePaddleIndex();
}

/////////////////////////////////////////////////////////////////////
// PROTECTED FUNCTIONS LIVE UNDER HERE
/////////////////////////////////////////////////////////////////////

void HydraInteractionTool::updatePaddleIndex()
{
  if(paddle_side_ == HYDRA_RIGHT) paddle_index_ = razer_hydra::Hydra::RIGHT;
  else if(paddle_side_ == HYDRA_LEFT) paddle_index_ = razer_hydra::Hydra::LEFT;
}

void HydraInteractionTool::updateFromMsg(const razer_hydra::HydraConstPtr &calib)
{
  ROS_DEBUG_NAMED("hydra", "Got hydra update!");

  razer_hydra::HydraPaddle paddle = calib->paddles[paddle_index_];

  // Update pose info
  tf::Transform interaction_handle;
  tf::transformMsgToTF(paddle.transform, interaction_handle);
  interaction_handle.setOrigin(interaction_handle.getOrigin()*workspace_radius_);
  handle_->setTransform(interaction_handle);


//  if(getToolButtonCount() < paddle.buttons.size())
//      setToolButtonCount(paddle.buttons.size());

  // Update the actual buttons
  for(size_t i = 0; i < 7; ++i)
  {
    setToolButtonState(i, paddle.buttons[i]);
  }
  // Magic number!
  setToolButtonState(7 + 0, paddle.trigger > 0.9);
  setToolButtonState(7 + 1, paddle.joy[0] >  0.8);
  setToolButtonState(7 + 2, paddle.joy[0] < -0.8);
  setToolButtonState(7 + 3, paddle.joy[1] >  0.8);
  setToolButtonState(7 + 4, paddle.joy[1] < -0.8);
}



}  // namespace something

