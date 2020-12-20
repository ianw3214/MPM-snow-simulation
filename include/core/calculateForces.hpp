#pragma once

#include <vector>
#include "particle.hpp"
#include "grid.hpp"

// Compute grid forces
// Inputs
//  - p: list of particles
// Outputs
//  - grid: particle data distributed onto a grid
void CalculateForces(std::vector<Particle>& p, Grid& grid);