#ifndef POINTCLOUD_UNDISTORTION__SO3_UTILS_HPP_
#define POINTCLOUD_UNDISTORTION__SO3_UTILS_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pointcloud_undistortion
{

class SO3
{
public:
  SO3()
  : rotation_(Eigen::Matrix3d::Identity())
  {
  }

  explicit SO3(const Eigen::Matrix3d & rotation)
  : rotation_(rotation)
  {
  }

  const Eigen::Matrix3d & matrix() const
  {
    return rotation_;
  }

  SO3 operator*(const SO3 & other) const
  {
    return SO3(rotation_ * other.rotation_);
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d & vector) const
  {
    return rotation_ * vector;
  }

  static Eigen::Matrix3d hat(const Eigen::Vector3d & omega)
  {
    Eigen::Matrix3d omega_hat;
    omega_hat <<
      0.0, -omega.z(), omega.y(),
      omega.z(), 0.0, -omega.x(),
      -omega.y(), omega.x(), 0.0;
    return omega_hat;
  }

  static SO3 exp(const Eigen::Vector3d & omega)
  {
    const double theta = omega.norm();
    if (theta < 1.0e-12) {
      return SO3(Eigen::Matrix3d::Identity() + hat(omega));
    }

    const Eigen::Vector3d axis = omega / theta;
    const Eigen::AngleAxisd angle_axis(theta, axis);
    return SO3(angle_axis.toRotationMatrix());
  }

  Eigen::Vector3d log() const
  {
    Eigen::AngleAxisd angle_axis(rotation_);
    return angle_axis.axis() * angle_axis.angle();
  }

private:
  Eigen::Matrix3d rotation_;
};

}  // namespace pointcloud_undistortion

#endif  // POINTCLOUD_UNDISTORTION__SO3_UTILS_HPP_
