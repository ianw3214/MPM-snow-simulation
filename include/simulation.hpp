#pragma once

#include "constants.hpp"
#include "grid.hpp"
#include "particle.hpp"
#include <memory>
#include <vector>

class Simulation {
public:
  explicit Simulation(const Constants &constants);

  void Update(double dt);

  const Constants constants;

  std::vector<Particle> particles;
  std::vector<std::unique_ptr<CollisionObject>> collision_objects;
  Grid grid;

  void add_collision_object(std::unique_ptr<CollisionObject> ptr);

private:
  bool m_firstTick;
};
