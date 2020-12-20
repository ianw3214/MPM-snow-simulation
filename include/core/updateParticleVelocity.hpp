#pragma once

#include "grid.hpp"
#include "particle.hpp"

#include <constants.hpp>
#include <vector>

// Update particle velocities based on grid node velocities
// Inputs
//  - grid: grid with nodes containing velocity data
// Outputs
//  - particles: list of particles to have velocity updated
void UpdateParticleVelocity(const Constants &constants,
                            std::vector<Particle> &particles, const Grid &grid);
