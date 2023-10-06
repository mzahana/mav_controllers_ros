#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>

// Ref: https://github.com/Jaeyoung-Lim/mavros_controllers/blob/master/geometric_controller/include/geometric_controller/common.h

static Eigen::Matrix3f quat2RotMatrix(const Eigen::Vector4f &q) {
    Eigen::Quaternionf quaternion(q(0), q(1), q(2), q(3));
    quaternion.normalize();
    Eigen::Matrix3f rotmat = quaternion.toRotationMatrix();
    // rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
    //     2 * q(0) * q(2) + 2 * q(1) * q(3),

    //     2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    //     2 * q(2) * q(3) - 2 * q(0) * q(1),

    //     2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
    //     q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

static Eigen::Vector4f rot2Quaternion(const Eigen::Matrix3f &R) {
   Eigen::Quaternionf quaternion(R);
   Eigen::Vector4f quat;
  quat << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
  // Eigen::Vector4f quat;
  // float tr = R.trace();
  // if (tr > 0.0) {
  //   float S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
  //   quat(0) = 0.25 * S;
  //   quat(1) = (R(2, 1) - R(1, 2)) / S;
  //   quat(2) = (R(0, 2) - R(2, 0)) / S;
  //   quat(3) = (R(1, 0) - R(0, 1)) / S;
  // } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
  //   float S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
  //   quat(0) = (R(2, 1) - R(1, 2)) / S;
  //   quat(1) = 0.25 * S;
  //   quat(2) = (R(0, 1) + R(1, 0)) / S;
  //   quat(3) = (R(0, 2) + R(2, 0)) / S;
  // } else if (R(1, 1) > R(2, 2)) {
  //   float S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
  //   quat(0) = (R(0, 2) - R(2, 0)) / S;
  //   quat(1) = (R(0, 1) + R(1, 0)) / S;
  //   quat(2) = 0.25 * S;
  //   quat(3) = (R(1, 2) + R(2, 1)) / S;
  // } else {
  //   float S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
  //   quat(0) = (R(1, 0) - R(0, 1)) / S;
  //   quat(1) = (R(0, 2) + R(2, 0)) / S;
  //   quat(2) = (R(1, 2) + R(2, 1)) / S;
  //   quat(3) = 0.25 * S;
  // }
  return quat;
}

static Eigen::Matrix3f matrix_hat(const Eigen::Vector3f &v) {
  Eigen::Matrix3f m;
  // Sanity checks on M
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}

static Eigen::Vector3f matrix_hat_inv(const Eigen::Matrix3f &m) {
  Eigen::Vector3f v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

static Eigen::Vector4f quatMultiplication(Eigen::Vector4f& q, Eigen::Vector4f& p)
{
    Eigen::Vector4f quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}

static Eigen::Vector4f quatFromEulerZYX(float yaw, float pitch, float roll)
{
  float cy = std::cos(yaw * 0.5);
  float sy = std::sin(yaw * 0.5);
  float cp = std::cos(pitch * 0.5);
  float sp = std::sin(pitch * 0.5);
  float cr = std::cos(roll * 0.5);
  float sr = std::sin(roll * 0.5);

  Eigen::Vector4f q;

  q(0) = cr * cp * cy + sr * sp * sy;  // w
  q(1) = sr * cp * cy - cr * sp * sy;  // x
  q(2) = cr * sp * cy + sr * cp * sy;  // y
  q(3) = cr * cp * sy - sr * sp * cy;  // z

  return q;
}

static Eigen::Vector4f inverseQuaternion(const Eigen::Vector4f& q) {
    float magnitudeSquared = q.squaredNorm();
    
    Eigen::Vector4f qConjugate(q(0), -q(1), -q(2), -q(3));
    
    return qConjugate / magnitudeSquared;
}

#endif