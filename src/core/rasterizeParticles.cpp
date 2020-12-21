#include "core/rasterizeParticles.hpp"
#include <cmath>

namespace {
double N(double x) {
  const double x_abs = std::abs(x);
  if (x_abs < 1)
    return std::pow(x_abs, 3) / 2.0 - std::pow(x_abs, 2) + 2.0 / 3.0;
  else if (x_abs < 2)
    return -std::pow(x_abs, 3) / 6.0 + std::pow(x_abs, 2) - 2.0 * x_abs +
           4.0 / 3.0;
  else
    return 0.0;
};

double dN(double x) {
  const double x_abs = std::abs(x);
  if (x_abs < 1)
    return x * x_abs * 3.0 / 2.0 - 2.0 * x;
  else if (x_abs < 2)
    return -x * x_abs / 2.0 + 2.0 * x - 2.0 * x / x_abs;
  else
    return 0.0;
}

double calculate_weight(double h_inverse, double x_offset, double y_offset,
                        double z_offset) {
  return N(h_inverse * x_offset) * N(h_inverse * y_offset) *
         N(h_inverse * z_offset);
};

double calculate_derivative(double h_inverse, double d_offset, double offset1,
                            double offset2) {
  return dN(h_inverse * d_offset) * N(h_inverse * offset1) *
         N(h_inverse * offset2) * h_inverse;
}
} // namespace

void RasterizeParticles(std::vector<Particle> &p, Grid &grid,
                        bool calculateVolumes) {
  // Common numbers used in calculations
  const double h_inverse = 1.0 / grid.cell_size();

  for (Particle &particle : p) {
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

    // When a particle lies on a grid node, the weights at the endpoints are 0
    // So we only have to worry about 4 grid nodes max per particle
    for (int a = -1; a < 3; ++a) {
      for (int b = -1; b < 3; ++b) {
        for (int c = -1; c < 3; ++c) {
          const int grid_x = x_index + a;
          const int grid_y = y_index + b;
          const int grid_z = z_index + c;
          const double x_offset = particle_x - grid_x * grid.cell_size();
          const double y_offset = particle_y - grid_y * grid.cell_size();
          const double z_offset = particle_z - grid_z * grid.cell_size();
          const double weight =
              calculate_weight(h_inverse, x_offset, y_offset, z_offset);
          grid.AppendMass(grid_x, grid_y, grid_z, particle_mass * weight);
          // Cache the weights to avoid calculating again when normalizing
          // velocities
          particle.m_weights[GridCoordinate{grid_x, grid_y, grid_z}] = weight;
          { // Calculate the derivative of the particle weight
            Eigen::Vector3d d_w = Eigen::Vector3d::Zero();
            // calculate it's derivative and cache it
            d_w.setIdentity();
            d_w(0) =
                calculate_derivative(h_inverse, x_offset, y_offset, z_offset);
            d_w(1) =
                calculate_derivative(h_inverse, y_offset, x_offset, z_offset);
            d_w(2) =
                calculate_derivative(h_inverse, z_offset, x_offset, y_offset);
            particle
                .m_weight_derivatives[GridCoordinate{grid_x, grid_y, grid_z}] =
                d_w;
          }
        }
      }
    }
  }
  // Calculate the velocities after the masses are calculated to normalize
  // properly
  unsigned int cache_index = 0;
  for (Particle &particle : p) {
    if (calculateVolumes) {
      particle.m_volume = 0.0;
      particle.m_density = 0.0;
    }
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
    for (int a = -1; a < 3; ++a) {
      for (int b = -1; b < 3; ++b) {
        for (int c = -1; c < 3; ++c) {
          const int grid_x = x_index + a;
          const int grid_y = y_index + b;
          const int grid_z = z_index + c;
          const double node_mass = grid.GetMass(grid_x, grid_y, grid_z);
          const double cached_weight =
              particle.m_weights[GridCoordinate{grid_x, grid_y, grid_z}];
          if (node_mass == 0.0 || cached_weight == 0.0)
            continue;
          grid.AppendVelocity(grid_x, grid_y, grid_z,
                              velocity * particle_mass * cached_weight /
                                  node_mass);
          // Also calculate the volume/density if needed
          if (calculateVolumes) {
            particle.m_density += node_mass * cached_weight;
          }
        }
      }
    }
    if (calculateVolumes) {
      particle.m_density /= std::pow(grid.cell_size(), 3);
      particle.m_volume = particle_mass / particle.m_density;
    }
  }
}
