#include "simulation.hpp"
#include "core/rasterizeParticles.hpp"
#include "update_deformation_gradient.hpp"

Simulation::Simulation(const Constants &constants)
    : constants(constants), m_firstTick(true) {}

void Simulation::Update(double dt) {
  // 1. Rasterize particle data to the grid.
  // 2. FIRST TIMESTEP ONLY: Compute particle volumes and densities.
  RasterizeParticles(particles, grid, m_firstTick);
  m_firstTick = false;

  // 3. Compute grid forces.

  // 4. Update velocities on grid.

  // 5. Grid-based body collisions.
  for (auto &collision_object : collision_objects) {
    grid.resolve_collision(*collision_object, dt);
  }

  // 6. Implicit velocity update.
  // Omitted (using explicit update).

  // 7. Update deformation gradients.
  auto get_grad_weight =
      [&](int i, int j, int k,
          const Eigen::Vector3d &position) -> Eigen::Vector3d {
    // TODO implement
    return Eigen::Vector3d::Zero();
  };
  for (auto &particle : particles) {
    update_deformation_gradient(constants, particle, grid, dt, get_grad_weight);
  }

  // 8. Update particle velocities.

  // 9. Particle-based body collisions.
  for (auto &collision_object : collision_objects) {
    for (auto &particle : particles) {
      particle.resolve_collision(*collision_object, dt);
    }
  }

  // 10. Update particle positions.
  for (auto &particle : particles) {
    particle.m_position += dt * particle.m_velocity;
  }

  // Clean up
  grid.clear();
}

void Simulation::add_collision_object(std::unique_ptr<CollisionObject> ptr) {
  collision_objects.emplace_back(std::move(ptr));
}
