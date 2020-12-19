#pragma once

#include <vector>
#include "particle.hpp"
#include "grid.hpp"

// Rasterize particle data to the grid
// Inputs
//  - p: list of particles
// Outputs
//  - grid: particle data distributed onto a grid
//
// *Particle list is not const reference because we will modify the volume on the first run
void RasterizeParticles(std::vector<Particle>& p, Grid& grid, bool calculateVolumes = false);