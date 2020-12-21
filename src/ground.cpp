#include "ground.hpp"

Ground::Ground() : Ground(0) {}

Ground::Ground(double y) : y(y) { friction_coeff = 0.35; }

bool Ground::detect_collision(const Eigen::Vector3d &position) {
  return position(1) <= y;
}

Eigen::Vector3d Ground::get_normal(const Eigen::Vector3d &position) {
  return Eigen::Vector3d(0, 1, 0);
}
