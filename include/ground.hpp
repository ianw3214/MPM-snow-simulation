#pragma once

#include "collision_object.hpp"

class Ground : public CollisionObject {
public:
  const double y;

  Ground();
  Ground(double y);
  bool detect_collision(const Eigen::Vector3d &position) override;
  Eigen::Vector3d get_normal(const Eigen::Vector3d &position) override;
};
