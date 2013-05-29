
#include <cat_user_entity/haptic_interaction_tool.h>
#include <chai3d_conversions/tf_chai.h>

#include <tf/tf.h>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <chai3d.h>



namespace something {


HapticInteractionTool::~HapticInteractionTool()
    {
        if(chai_device_handler_) delete chai_device_handler_;
        if(chai_device_)
        {
            chai_device_->setForceAndTorqueAndGripperForce(cVector3d(0,0,0), cVector3d(0,0,0), 0);
            chai_device_->close();
            delete chai_device_;
        }
    }


void HapticInteractionTool::init()
{

    cHapticDeviceInfo info = chai_device_->getSpecifications();
    cVector3d pos = info.m_positionOffset;
    setPosition(tf::Vector3(pos.x(), pos.y(), pos.z()));
    setQuaternion(tf::createQuaternionFromYaw(M_PI)*tf::createQuaternionFromRPY(0, 0.4, 0));

    std::string modelName = info.m_modelName;

    int button_count = 0;
    if(modelName == "PHANTOM Omni") button_count = 2;
    else if(modelName == "Falcon") button_count = 4;
    else if(modelName == "omega") button_count = 1;
    setToolButtonCount(button_count);
    // TODO this should come from a config file!
    button_name_map_["click"] = 0;
    button_name_map_["menu"] = 1;

    workspace_radius_ = workspace_radius_ / info.m_workspaceRadius;
    k_linear_ = 0.06*info.m_maxLinearStiffness / workspace_radius_;
    k_angular_ = 0;

    // TODO this should be a chai thread...!
    ros::NodeHandle nh;
    float update_period = 0.01;
    interaction_timer_ = nh.createTimer(ros::Duration(update_period), boost::bind( &HapticInteractionTool::updateDevice, this ) );
}


/////////////////////////////////////////////////////////////////////
// PROTECTED FUNCTIONS LIVE UNDER HERE
/////////////////////////////////////////////////////////////////////

void HapticInteractionTool::updateButtonStates()
{
    for(size_t i = 0; i < getToolButtonCount(); ++i)
    {
        bool state = false;
        chai_device_->getUserSwitch(i, state );
        setToolButtonState(i, state);
    }
}

// Call an update?
void HapticInteractionTool::updateDevice()
{

    // Get the state of all buttons
    updateButtonStates();

    // Get position and velocity info for the haptic handle

    // read position
    cVector3d position;
    chai_device_->getPosition(position);
    position = workspace_radius_*position;

    // read orientation
    cMatrix3d rotation;
    chai_device_->getRotation(rotation);

    // read gripper position
    double gripperAngle;
    chai_device_->getGripperAngleRad(gripperAngle);

    // read linear velocity
    cVector3d linearVelocity;
    chai_device_->getLinearVelocity(linearVelocity);
    linearVelocity = workspace_radius_*linearVelocity;

    // read angular velocity
    cVector3d angularVelocity;
    chai_device_->getAngularVelocity(angularVelocity);

    // read gripper angular velocity
    double gripperAngularVelocity;
    chai_device_->getGripperAngularVelocity(gripperAngularVelocity);

    tf::Transform haptic_handle, interaction_handle;
    haptic_handle = interaction_handle = tf::Transform::getIdentity();
    haptic_handle = tf::Transform(tf::matrixChaiToTf(rotation), tf::vectorChaiToTf(position));
    //if(getToolButtonState(0))
        interaction_handle = haptic_handle*tf::Transform(tf::createQuaternionFromRPY(0,0,M_PI));
    //else
    //    interaction_handle = haptic_handle;
    handle_->setTransform(interaction_handle);

    //k_linear = chai_device_->getSpecifications().m_maxLinearStiffness * 0.5;
    updateVirtualCoupling();

    cVector3d force = tf::vectorTfToChai(last_tool_force_);
    cVector3d torque = tf::vectorTfToChai(last_tool_torque_);

    float gripperForce = 0;

    // send computed force, torque and gripper force to haptic device
    chai_device_->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

}



}  // namespace something

