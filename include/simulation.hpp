#pragma once

#include "constants.hpp"
#include "grid.hpp"
#include "particle.hpp"
#include <vector>

class Simulation {
public:
  explicit Simulation(const Constants &constants);

  void Update(double dt);

  const Constants constants;

  std::vector<Particle> particles;
  Grid grid;

private:
};
