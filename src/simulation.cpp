#include "simulation.hpp"
#include "core/calculateForces.hpp"
#include "core/rasterizeParticles.hpp"
#include "core/updateGridVelocity.hpp"
#include "core/updateParticleVelocity.hpp"
#include "update_deformation_gradient.hpp"

Simulation::Simulation(const Constants &constants)
    : constants(constants), m_firstTick(true) {}

void Simulation::Init(double size, unsigned int edge_length, Eigen::Vector3d p)
{
  const double granularity = size / static_cast<double>(edge_length);

  particles.clear();
  grid.clear();
  for (unsigned int x = 0; x < edge_length; ++x) {
    for (unsigned int y = 0; y < edge_length; ++y) {
      for (unsigned int z = 0; z < edge_length; ++z) {
        particles.emplace_back(
          p + Eigen::Vector3d(granularity * x, granularity * y, granularity * z),
          Eigen::Vector3d::Zero()
        );
      }
    }
  }
}

void Simulation::Update(double dt) {
  // 1. Rasterize particle data to the grid.
  // 2. FIRST TIMESTEP ONLY: Compute particle volumes and densities.
  RasterizeParticles(particles, grid, m_firstTick);
  m_firstTick = false;

  // 3. Compute grid forces.
  CalculateForces(constants, particles, grid);

  // 4. Update velocities on grid.
  UpdateGridVelocity(grid, dt);

  // 5. Grid-based body collisions.
  for (auto &collision_object : collision_objects) {
    grid.resolve_collision(*collision_object, dt);
  }

  // 6. Implicit velocity update.
  // Omitted (using explicit update).

  // 7. Update deformation gradients.
  auto get_grad_weight = [&](const GridCoordinate &coord,
                             const Particle &particle) -> Eigen::Vector3d {
    auto it = particle.m_weight_derivatives.find(coord);
    return it == particle.m_weight_derivatives.end() ? Eigen::Vector3d::Zero()
                                                     : it->second;
  };
  for (auto &particle : particles) {
    update_deformation_gradient(constants, particle, grid, dt, get_grad_weight);
  }

  // 8. Update particle velocities.
  UpdateParticleVelocity(constants, particles, grid);

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
  for (Particle &particle : particles) {
    particle.ResetWeightsCache();
  }
}

void Simulation::add_collision_object(std::unique_ptr<CollisionObject> ptr) {
  collision_objects.emplace_back(std::move(ptr));
}
