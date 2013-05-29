



namespace something {

typedef Vector3 Eigen::Vector3f;
typedef Quaternion Eigen::Quaternion;

class AbstractInteractionTool{


public:
// Methods only!
  
  // Read the position of the tool.
  Eigen::Vector3f getToolPosition() const;
  
  // Read the quaternion of the tool.
  Eigen::Quaternion getToolQuaternion() const;

  // Read the state of the binary switches on the tool.
  bool getToolButtonState(const unsigned int &index) const;

  // Get the number of buttons available on the tool.
  unsigned int getToolButtonCount() const;
  
//! Get the name of this tool.
  std::string getToolName() const;

  // Set the force applied to the tool
  void setToolForce(const Eigen::Vector3f &force);

  // Set the torque applied to the tool
  void setToolTorque(const Eigen::Vector3f &torque);
  
  // Set the force and torque applied to the tool
  void setToolForceAndTorque(const Eigen::Vector3f &force, const Eigen::Vector3f &torque);
  
  // Set the gripper force on the tool
  void setToolGripperForce(const float &force);

  //! Device's base frame is transparent, client sees only a device with "infinite workspace"
  // Set tool position
  void setToolPosition(const Eigen::Vector3f &position);
  
  // Set tool quaternion
  void setToolQuaternion(const Eigen::Quaternion &quaternion);

  // Set workspace size for the tool interaction
  void setToolWorkspaceRadius(const float &radius);


  


protected: 
// Methods



// Members


};





}  // namespace something
