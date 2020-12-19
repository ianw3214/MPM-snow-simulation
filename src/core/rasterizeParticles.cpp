#include "core/rasterizeParticles.hpp"
#include <cmath>

void RasterizeParticles(const std::vector<Particle> &p, Grid &grid) {
  // Common numbers used in calculations
  const double h_inverse = 1.0 / grid.cell_size();

  // Cache the weights to avoid calculating again when normalizing velocities
  std::vector<double> weights_cache;
  weights_cache.reserve(p.size() * 4 * 4 * 4);

  for (const Particle &particle : p) {
    const double particle_x = particle.m_position(0);
    const double particle_y = particle.m_position(1);
    const double particle_z = particle.m_position(2);
    const double particle_mass = particle.m_mass;
    // Negative particles need to get rounded 'down'
    const int x_index =
        static_cast<int>(std::floor(particle_x / grid.cell_size()));
    const int y_index =
        static_cast<int>(std::floor(particle_y / grid.cell_size()));
    const int z_index =
        static_cast<int>(std::floor(particle_z / grid.cell_size()));

    auto calculate_weight = [=](double x_offset, double y_offset,
                                double z_offset) {
      auto N = [=](double x) {
        const double x_abs = std::abs(x);
        if (x_abs < 1)
          return std::pow(x_abs, 3) / 2.0 - std::pow(x_abs, 2) + 2.0 / 3.0;
        else if (x_abs < 2)
          return -std::pow(x_abs, 3) / 6.0 + std::pow(x_abs, 2) - 2.0 * x_abs +
                 4.0 / 3.0;
        else
          return 0.0;
      };
      return N(h_inverse * x_offset) * N(h_inverse * y_offset) *
             N(h_inverse * z_offset);
    };
    // When a particle lies on a grid node, the weights at the endpoints are 0
    // So we only have to worry about 4 grid nodes max per particle
    for (int a = 0; a < 4; ++a) {
      for (int b = 0; b < 4; ++b) {
        for (int c = 0; c < 4; ++c) {
          const int grid_x = x_index + a;
          const int grid_y = y_index + b;
          const int grid_z = z_index + c;
          const double weight = calculate_weight(
              particle_x - grid_x, particle_y - grid_y, particle_z - grid_z);
          grid.AppendMass(grid_x, grid_y, grid_z, particle_mass * weight);
          // Just push without indexing since velocity calculations are done in
          // the same order
          weights_cache.push_back(weight);
        }
      }
    }
  }
  // Calculate the velocities after the masses are calculated to normalize
  // properly
  unsigned int cache_index = 0;
  for (const Particle &particle : p) {
    const double particle_x = particle.m_position(0);
    const double particle_y = particle.m_position(1);
    const double particle_z = particle.m_position(2);
    const Eigen::Vector3d &velocity = particle.m_velocity;
    const double particle_mass = particle.m_mass;
    // Negative particles need to get rounded 'down'
    const int x_index =
        static_cast<int>(std::floor(particle_x / grid.cell_size()));
    const int y_index =
        static_cast<int>(std::floor(particle_y / grid.cell_size()));
    const int z_index =
        static_cast<int>(std::floor(particle_z / grid.cell_size()));

    // When a particle lies on a grid node, the weights at the endpoints are 0
    // So we only have to worry about 4 grid nodes max per particle
    for (int a = 0; a < 4; ++a) {
      for (int b = 0; b < 4; ++b) {
        for (int c = 0; c < 4; ++c) {
          const int grid_x = x_index + a;
          const int grid_y = y_index + b;
          const int grid_z = z_index + c;
          const double node_mass = grid.GetMass(grid_x, grid_y, grid_z);
          const double cached_weight = weights_cache[cache_index++];
          grid.AppendVelocity(grid_x, grid_y, grid_z,
                              velocity * particle_mass * cached_weight /
                                  node_mass);
        }
      }
    }
  }
}
