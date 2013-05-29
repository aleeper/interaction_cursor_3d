#ifndef _CAT_HAPTIC_INTERACTION_TOOL_H_
#define _CAT_HAPTIC_INTERACTION_TOOL_H_

#include <cat_user_entity/abstract_interaction_tool.h>

#include <chai3d_conversions/tf_chai.h>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <chai3d.h>


namespace something {

class HapticInteractionTool: public AbstractInteractionTool{


public:
  // Methods only!

    HapticInteractionTool(const std::string &frame_id,
                                                 tf::TransformListener *tfl,
                                                 tf::TransformBroadcaster *tfb)
        : AbstractInteractionTool(frame_id, tfl, tfb),
          chai_device_handler_(0), chai_device_(0), workspace_radius_(0.25)
    {
        // Must come first, and must be defined in the header due to library issues in CHAI3D.
        initializeHaptics();

        // Finish other intialization stuff.
        init();
    }

    virtual ~HapticInteractionTool();

    // Must be defined in the header due to library issues in CHAI3D.
    inline void initializeHaptics()
    {
        if(!chai_device_handler_)
            chai_device_handler_ = new cHapticDeviceHandler();
        chai_device_handler_->getDevice(chai_device_, 0);

        // open a connection with the Shaptic device
        if(chai_device_->open() != 0)
        {
            ROS_ERROR("Error opening chai device!");
            return;
        }
    }

    void init();



////  // Set the force applied to the tool
////  virtual void setToolForce(const Vector3 &force)           { last_tool_force_ = force; }

////  // Set the torque applied to the tool
////  virtual void setToolTorque(const Vector3 &torque)         { last_tool_torque_ = torque; }

////  // Set the force and torque applied to the tool
////  virtual void setToolForceAndTorque(const Vector3 &force, const Vector3 &torque) { setToolForce(force); setToolTorque(torque); }

////  // Set the gripper force on the tool
////  virtual void setToolGripperForce(const float &force);

////  //! Device's base frame is transparent, client sees only a device with "infinite workspace"
////  // Set tool position
////  virtual void setToolPosition(const Vector3 &position)
////  {
////    cVector3d device_position;
////    chai_device_->getPosition(device_position);
////    base_frame_position_ = position - base_frame_quaternion_ * device_position;
////  }

////  // Set tool quaternion
////  virtual void setToolQuaternion(const Quaternion &quaternion)
////  {
////    cMatrix3d device_matrix;
////    chai_device_->getRotation(device_matrix);
////    Quaternion device_quaternion(device_matrix);
////    base_frame_quaternion_ = quaternion * device_quaternion.conjugate();
////  }

////  // Set tool position
////  virtual void setInteractionFramePosition(const Vector3 &position)
////  {
////    base_frame_position_ = position;
////  }

////  // Set tool quaternion
////  virtual void setInteractionFrameQuaternion(const Quaternion &quaternion)
////  {
////    base_frame_quaternion_ = quaternion;
////  }

////  // Set workspace size for the tool interaction
////  virtual void setToolWorkspaceRadius(const float &radius);



protected:
// Methods

//  // Call an update?
  virtual void updateDevice();

    void updateButtonStates();

// Members

  cHapticDeviceHandler *chai_device_handler_;
  cGenericHapticDevice *chai_device_;

  ros::Timer interaction_timer_;

  float workspace_radius_;

};


}  // namespace something

#endif
