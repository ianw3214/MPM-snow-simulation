#pragma once

#include "grid.hpp"

// Update grid node velocities based on forces
// Inputs
//  - grid: grid with nodes containing force data and input velocity
//  - dt: descrete delta time between updates
// Outputs
//  - grid: grid with nodes containing final velocity data
void UpdateGridVelocity(Grid& grid, double dt);