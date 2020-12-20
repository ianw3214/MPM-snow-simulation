#pragma once

#include <unordered_map>
#include "grid.hpp"

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
  Particle(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double m = 0.0, double vol = 0.0, double d = 0.0);

  void resolve_collision(CollisionObject &collision_object, double dt);

  // Cache weights
  inline void ResetWeightsCache() {
	  m_weights.clear();
	  m_weight_derivatives.clear();
  };
  std::unordered_map<GridCoordinate, double> m_weights;
  std::unordered_map<GridCoordinate, Eigen::Vector3d> m_weight_derivatives;

private:
};
