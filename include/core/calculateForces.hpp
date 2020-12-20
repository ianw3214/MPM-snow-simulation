#pragma once

#include <vector>
#include "particle.hpp"
#include "grid.hpp"

#include "constants.hpp"

// Compute grid forces
// Inputs
//  - constants: constant values used in calculations
//  - p: list of particles
// Outputs
//  - p: particles with updated weight derivatives
//  - grid: per grid node forces calculated from particle data
void CalculateForces(const Constants& constants, std::vector<Particle>& p, Grid& grid, double dt);