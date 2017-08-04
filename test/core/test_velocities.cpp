#include <iostream>

#include <sophus/velocities.hpp>
#include "tests.hpp"

namespace Sophus {

template <class Scalar>
void tests() {
  std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> bar_Ts_baz =
      getTestSE3s<Scalar>();
  SE3<Scalar> foo_T_bar =
      SE3<Scalar>::rotX(0.5) * SE3<Scalar>::rotZ(0.2) * SE3<Scalar>::transY(2);

  std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> foo_Ts_baz;
  for (auto const& bar_T_baz : bar_Ts_baz) {
    foo_Ts_baz.push_back(foo_T_bar * bar_T_baz);
  }

  auto gen_lin_vels = [](
      std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> const&
          bar_Ts_baz) {
    std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
        linearVels_foo;
    for (size_t i = 0; i < bar_Ts_baz.size() - 1; ++i) {
      linearVels_foo.push_back(bar_Ts_baz[i + 1].translation() -
                               bar_Ts_baz[i].translation());
    }
    return linearVels_foo;
  };

  auto gen_rot_vels = [](
      std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> const&
          bar_Ts_baz) {
    std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
        rotVels_foo;
    for (size_t i = 0; i < bar_Ts_baz.size() - 1; ++i) {
      rotVels_foo.push_back(
          (bar_Ts_baz[i].so3().inverse() * bar_Ts_baz[i + 1].so3()).log());
    }
    return rotVels_foo;
  };

  std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
      linearVels_foo = gen_lin_vels(foo_Ts_baz);
  for (auto const& v_foo : linearVels_foo) {
    std::cerr << v_foo.transpose() << std::endl;
  }
  std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
      linearVels_bar = gen_lin_vels(bar_Ts_baz);
  for (auto const& v_bar : linearVels_bar) {
    std::cerr << transformLinearVelocity(foo_T_bar, v_bar).transpose()
              << std::endl;
  }
  std::cerr << "rotVels_foo" << std::endl;
  std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
      rotVels_foo = gen_rot_vels(foo_Ts_baz);
  for (auto const& omega_foo : rotVels_foo) {
    std::cerr << omega_foo.transpose() << std::endl;
  }
  std::cerr << "rotVels_bar" << std::endl;

  std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
      rotVels_bar = gen_rot_vels(bar_Ts_baz);
  for (auto const& v_bar : rotVels_bar) {
    std::cerr << transformRotVelocity(foo_T_bar, v_bar).transpose()
              << std::endl;
  }
}

int test_velocities() {
  using std::cerr;
  using std::endl;

  cerr << "Test Velocities" << endl << endl;
  cerr << "Double tests: " << endl;
  tests<double>();
  cerr << "Float tests: " << endl;
  tests<float>();
  return 0;
}
}  // namespace Sophus

int main() { return Sophus::test_velocities(); }
