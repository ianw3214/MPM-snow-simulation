#pragma once

#include "collision_object.hpp"
#include <Eigen/Core>

class Particle {
public:
  Eigen::Vector3d m_position;
  Eigen::Vector3d m_velocity;

  double m_mass;
  double m_volume;
  double m_density;

  /**
   * Deformation gradients
   */
  Eigen::Matrix3d m_def_elastic, m_def_plastic;

  Particle();

  void resolve_collision(CollisionObject &collision_object, double dt);

private:
};
