#include "ground.hpp"

Ground::Ground() : Ground(0) {}

Ground::Ground(double z) : z(z) { friction_coeff = 0.1; }

bool Ground::detect_collision(const Eigen::Vector3d &position) {
  return position(2) <= z;
}

Eigen::Vector3d Ground::get_normal(const Eigen::Vector3d &position) {
  return Eigen::Vector3d(0, 0, 1);
}
