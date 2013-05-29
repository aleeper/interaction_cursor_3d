
#include <haptic_sandbox/abstract_interaction_tool.h>

#include <Eigen/Geometry>
#include <chai3d.h>

#include <ros/ros.h>



namespace something {

typedef Eigen::Vector3d Vector3;
typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Transformd Transform;

class AbstractCoupling{


    setProxyFramePosition(const Vector3 &position);
    setProxyFrameQuaternion(const Quaternion &quaternion);
    setProxyFrameTransform(const Transform &transform);

    Vector3 getProxyFramePosition() const;
    Quaternion getProxyFrameQuaternion() const;
    Transform getProxyFrameTransform() const;

    zeroCouplingByMovingProxy();
    zeroCouplingByMovingTool();

    setCouplingStiffness(const float &k);
    setCouplingDamping(const float &b);





protected:


    // Members
    Vector3 proxy_frame_position_;
    Quaternion proxy_frame_position_;
    Transform proxy_frame_transform_;

    float stiffness_;
    float damping_;

};


} // namespace something
