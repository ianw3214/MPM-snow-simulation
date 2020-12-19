#pragma once

#include <vector>
#include "particle.hpp"
#include "grid.hpp"

// Rasterize particle data to the grid
// Inputs
//  - p: list of particles
// Outputs
//  - grid: particle data distributed onto a grid
void RasterizeParticles(const std::vector<Particle>& p, Grid& grid);