#pragma once

#include "collision_object.hpp"
#include <Eigen/Core>

class Particle {
public:
  Eigen::Vector3d m_position;
  Eigen::Vector3d m_velocity;

  float m_mass;
  Eigen::Matrix3d m_deformation;

  void resolve_collision(CollisionObject &collision_object, double dt);

private:
};
