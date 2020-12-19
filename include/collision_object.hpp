#pragma once

#include <Eigen/Dense>

class CollisionObject {
public:
  virtual ~CollisionObject() = default;

  double friction_coeff = 0;

  virtual bool detect_collision(const Eigen::Vector3d &position) = 0;
  virtual Eigen::Vector3d get_normal(const Eigen::Vector3d &position) = 0;
  virtual void resolve_collision(const Eigen::Vector3d &position,
                                 Eigen::Vector3d &velocity, double dt);
};
