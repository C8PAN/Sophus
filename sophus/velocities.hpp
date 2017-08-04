#ifndef SOPHUS_VELOCITIES_HPP
#define SOPHUS_VELOCITIES_HPP

#include "se3.hpp"

namespace Sophus {

template <class Scalar>
Vector3<Scalar> transformLinearVelocity(SO3<Scalar> const foo_R_bar,
                                        Vector3<Scalar> const& linearVel_bar) {
  return foo_R_bar * linearVel_bar;
}

template <class Scalar>
Vector3<Scalar> transformLinearVelocity(SE3<Scalar> const foo_T_bar,
                                        Vector3<Scalar> const& linearVel_bar) {
  return transformLinearVelocity(foo_T_bar.so3(), linearVel_bar);
}

template <class Scalar>
Vector3<Scalar> transformRotVelocity(SO3<Scalar> const foo_R_bar,
                                     Vector3<Scalar> const& rotVecl_bar) {
  return SO3<Scalar>::vee(foo_R_bar.matrix() * SO3<Scalar>::hat(rotVecl_bar) *
                          foo_R_bar.inverse().matrix());
}

template <class Scalar>
Vector3<Scalar> transformRotVelocity(SE3<Scalar> const foo_T_bar,
                                     Vector3<Scalar> const& rotVecl_bar) {
  return transformRotVelocity(foo_T_bar.so3(), rotVecl_bar);
}

}  // namespace Sophus

#endif  // SOPHUS_VELOCITIES_HPP
