#include "simulation.hpp"
#include "core/rasterizeParticles.hpp"

Simulation::Simulation(const Constants &constants) : constants(constants) {}

void Simulation::Update(double dt) {
  // 1. Rasterize particle data to the grid.
  RasterizeParticles(particles, grid);

  // 2. FIRST TIMESTEP ONLY: Compute particle volumes and densities.

  // 3. Compute grid forces.

  // 4. Update velocities on grid.

  // 5. Grid-based body collisions.
  for (auto &collision_object : collision_objects) {
    grid.resolve_collision(*collision_object, dt);
  }

  // 6. Time integration.

  // 7. Update deformation gradient.

  // 8. Update particle velocities.

  // 9. Particle-based body collisions.
  for (auto &collision_object : collision_objects) {
    for (auto &particle : particles) {
      particle.resolve_collision(*collision_object, dt);
    }
  }

  // 10. Update particle positions.

  // Clean up
  grid.clear();
}

void Simulation::add_collision_object(std::unique_ptr<CollisionObject> ptr) {
  collision_objects.emplace_back(std::move(ptr));
}
