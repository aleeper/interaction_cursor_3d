

#include <Eigen/Geometry>



namespace something {

typedef Eigen::Vector3d Vector3;
typedef Eigen::Quaterniond Quaternion;

class ToolEntity{


public:
  // Methods only!

  // Constructor
  ToolEntity() {}

//  virtual Vector3 getPosition() const           { return workspace_transform_.translation(); }
//  virtual Quaternion getQuaternion() const      { return workspace_quaternion_.rotation(); }
//  virtual Transform getTransform() const      { return workspace_transform_; }

//  virtual void setWorkspacePosition(const Vector3 &position);
//  virtual void setWorkspaceQuaternion(const Quaternion &quaternion);
//  virtual void setWorkspaceTransform(const Transform &transform);

  void attachCoupling()
  {


  }




protected:

};





}  // namespace something
