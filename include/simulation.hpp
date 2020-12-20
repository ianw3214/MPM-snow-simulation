#pragma once

#include <Eigen/Core>

#include "constants.hpp"
#include "grid.hpp"
#include "particle.hpp"
#include <memory>
#include <vector>

class Simulation {
public:
  explicit Simulation(const Constants &constants);

  void Init(double size = 1.0, unsigned int edge_length = 10, Eigen::Vector3d p = Eigen::Vector3d::Zero());
  void Update(double dt);

  const Constants constants;

  std::vector<Particle> particles;
  std::vector<std::unique_ptr<CollisionObject>> collision_objects;
  Grid grid;

  void add_collision_object(std::unique_ptr<CollisionObject> ptr);

private:
  bool m_firstTick;
};
