#ifndef POINTCLOUD_UNDISTORTION__IKFOM_HPP_
#define POINTCLOUD_UNDISTORTION__IKFOM_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include "pointcloud_undistortion/so3_utils.hpp"

namespace pointcloud_undistortion
{

constexpr double kGravity = 9.81;

struct StateIkfom
{
  Eigen::Vector3d pos{0.0, 0.0, 0.0};
  SO3 rot{Eigen::Matrix3d::Identity()};
  SO3 offset_R_L_I{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d offset_T_L_I{0.0, 0.0, 0.0};
  Eigen::Vector3d vel{0.0, 0.0, 0.0};
  Eigen::Vector3d bg{0.0, 0.0, 0.0};
  Eigen::Vector3d ba{0.0, 0.0, 0.0};
  Eigen::Vector3d grav{0.0, 0.0, -kGravity};
};

struct InputIkfom
{
  Eigen::Vector3d acc{0.0, 0.0, 0.0};
  Eigen::Vector3d gyro{0.0, 0.0, 0.0};
};

inline Eigen::Matrix<double, 12, 12> processNoiseCov()
{
  Eigen::Matrix<double, 12, 12> q = Eigen::Matrix<double, 12, 12>::Zero();
  q.block<3, 3>(0, 0) = 0.0001 * Eigen::Matrix3d::Identity();
  q.block<3, 3>(3, 3) = 0.0001 * Eigen::Matrix3d::Identity();
  q.block<3, 3>(6, 6) = 0.00001 * Eigen::Matrix3d::Identity();
  q.block<3, 3>(9, 9) = 0.00001 * Eigen::Matrix3d::Identity();
  return q;
}

inline Eigen::Matrix<double, 24, 1> getF(const StateIkfom & state, const InputIkfom & input)
{
  Eigen::Matrix<double, 24, 1> f = Eigen::Matrix<double, 24, 1>::Zero();
  const Eigen::Vector3d omega = input.gyro - state.bg;
  const Eigen::Vector3d a_inertial = state.rot.matrix() * (input.acc - state.ba);

  f.block<3, 1>(0, 0) = state.vel;
  f.block<3, 1>(3, 0) = omega;
  f.block<3, 1>(12, 0) = a_inertial + state.grav;
  return f;
}

inline Eigen::Matrix<double, 24, 24> dfDx(const StateIkfom & state, const InputIkfom & input)
{
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d acc = input.acc - state.ba;

  cov.block<3, 3>(12, 3) = -state.rot.matrix() * SO3::hat(acc);
  cov.block<3, 3>(12, 18) = -state.rot.matrix();
  cov.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  cov.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return cov;
}

inline Eigen::Matrix<double, 24, 12> dfDw(const StateIkfom & state)
{
  Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
  cov.block<3, 3>(12, 3) = -state.rot.matrix();
  cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
  return cov;
}

class EsEkf
{
public:
  using Cov = Eigen::Matrix<double, 24, 24>;
  using VectorizedState = Eigen::Matrix<double, 24, 1>;

  StateIkfom getX() const { return x_; }
  Cov getP() const { return p_; }
  void changeX(const StateIkfom & input_state) { x_ = input_state; }
  void changeP(const Cov & input_cov) { p_ = input_cov; }

  StateIkfom boxplus(const StateIkfom & x, const VectorizedState & f) const
  {
    StateIkfom out;
    out.pos = x.pos + f.block<3, 1>(0, 0);
    out.rot = x.rot * SO3::exp(f.block<3, 1>(3, 0));
    out.offset_R_L_I = x.offset_R_L_I * SO3::exp(f.block<3, 1>(6, 0));
    out.offset_T_L_I = x.offset_T_L_I + f.block<3, 1>(9, 0);
    out.vel = x.vel + f.block<3, 1>(12, 0);
    out.bg = x.bg + f.block<3, 1>(15, 0);
    out.ba = x.ba + f.block<3, 1>(18, 0);
    out.grav = x.grav + f.block<3, 1>(21, 0);
    return out;
  }

  void predict(double dt, Eigen::Matrix<double, 12, 12> & q, const InputIkfom & input)
  {
    const Eigen::Matrix<double, 24, 1> f = getF(x_, input);
    Eigen::Matrix<double, 24, 24> f_x = dfDx(x_, input);
    const Eigen::Matrix<double, 24, 12> f_w = dfDw(x_);

    x_ = boxplus(x_, f * dt);
    f_x = Eigen::Matrix<double, 24, 24>::Identity() + f_x * dt;
    p_ = f_x * p_ * f_x.transpose() + (dt * f_w) * q * (dt * f_w).transpose();
  }

private:
  StateIkfom x_{};
  Cov p_{Cov::Identity()};
};

}  // namespace pointcloud_undistortion

#endif  // POINTCLOUD_UNDISTORTION__IKFOM_HPP_
