#include "simulation.hpp"
#include "core/rasterizeParticles.hpp"

Simulation::Simulation(const Constants &constants) : constants(constants) {}

void Simulation::Update(double dt) {
  RasterizeParticles(particles, grid);

  // TODO

  grid.clear();
}
