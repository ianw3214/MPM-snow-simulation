#include "simulation.hpp"
#include "core/calculateForces.hpp"
#include "core/rasterizeParticles.hpp"
#include "core/updateGridVelocity.hpp"
#include "core/updateParticleVelocity.hpp"
#include "image.hpp"
#include "update_deformation_gradient.hpp"
#include <chrono>
#include <cstdio>

namespace {
double map(double inStart, double inEnd, double outStart, double outEnd,
           double value) {
  return outStart + (value - inStart) * (outEnd - outStart) / (inEnd - inStart);
}

void draw_to_image(Image &image, int x, int y, float value) {
  if (x < 0 || x >= image.width || y < 0 || y >= image.height) {
    return;
  }
  image(x, y, 0) = value;
}
} // namespace

Simulation::Simulation(const Constants &constants)
    : grid(constants.grid_cell_size), constants(constants), m_firstTick(true),
      camera(Eigen::Vector3d(0, 2, -4), Eigen::Vector3d(0, 0, 1),
             Eigen::Vector3d(0, 1, 0), 1),
      image(256, 256, 1),
      gen(std::random_device()() ^
          ((std::mt19937::result_type)
               std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count() +
           (std::mt19937::result_type)
               std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now().time_since_epoch())
                   .count())) {}

void Simulation::add_sphere(double radius, unsigned int edge_length,
                            const Eigen::Vector3d &position,
                            const Eigen::Vector3d &velocity, double mass) {
  const double granularity = radius * 2.0 / static_cast<double>(edge_length);

  Eigen::Vector3d p(position(0) - radius, position(1) - radius,
                    position(2) - radius);

  std::uniform_real_distribution<double> distribution(-granularity * 0.2,
                                                      granularity * 0.2);

  for (unsigned int x = 0; x < edge_length; ++x) {
    for (unsigned int y = 0; y < edge_length; ++y) {
      for (unsigned int z = 0; z < edge_length; ++z) {
        Eigen::Vector3d pos =
            p + Eigen::Vector3d(granularity * x + distribution(gen),
                                granularity * y + distribution(gen),
                                granularity * z + distribution(gen));
        if ((pos - position).squaredNorm() > radius * radius) {
          continue;
        }
        particles.emplace_back(pos, velocity, mass);
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
  CalculateForces(constants, particles, grid, dt);

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
    update_deformation_gradient(constants, particle, grid, dt, get_grad_weight,
                                false);
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

  // Render
  image.clear();
  for (auto &particle : particles) {
    auto screen_pos = camera.project(particle.m_position);
    int x = ((int)((screen_pos(0) * 0.5 + 0.5) * (image.width - 1)));
    int y = ((int)((screen_pos(1) * 0.5 + 0.5) * (image.height - 1)));
    float value =
        std::clamp(map(0, render_depth, 1, 0, screen_pos(2)), 0.0, 1.0);
    draw_to_image(image, x, y, value);
    draw_to_image(image, x - 1, y, value);
    draw_to_image(image, x + 1, y, value);
    draw_to_image(image, x, y - 1, value);
    draw_to_image(image, x, y + 1, value);
  }
  char buff[100];
  snprintf(buff, sizeof(buff), "image%03d.jpg", frame_index);
  std::string filename = buff;
  image.save_to_file(filename);
  frame_index++;

  // Clean up
  grid.clear();
  for (Particle &particle : particles) {
    particle.ResetWeightsCache();
  }
}

void Simulation::add_collision_object(std::unique_ptr<CollisionObject> ptr) {
  collision_objects.emplace_back(std::move(ptr));
}
