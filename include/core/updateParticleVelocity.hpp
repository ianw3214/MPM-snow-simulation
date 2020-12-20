#pragma once

#include "particle.hpp"
#include "grid.hpp"

constexpr double alpha = 0.95;

// Update particle velocities based on grid node velocities
// Inputs
//  - grid: grid with nodes containing velocity data
// Outputs
//  - particles: list of particles to have velocity updated
void UpdateParticleVelocity(std::vector<Particle>& particles, const Grid& grid);