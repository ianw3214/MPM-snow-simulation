#include "core/updateParticleVelocity.hpp"

void UpdateParticleVelocity(const Constants &constants,
                            std::vector<Particle> &particles,
                            const Grid &grid) {
  Eigen::Vector3d temp_velocity;
  Eigen::Vector3d temp_old_velocity;

  for (Particle &particle : particles) {
    Eigen::Vector3d v_PIC = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_FLIP = particle.m_velocity;

    const double particle_x = particle.m_position(0);
    const double particle_y = particle.m_position(1);
    const double particle_z = particle.m_position(2);
    const int x_index =
        static_cast<int>(std::floor(particle_x / grid.cell_size()));
    const int y_index =
        static_cast<int>(std::floor(particle_y / grid.cell_size()));
    const int z_index =
        static_cast<int>(std::floor(particle_z / grid.cell_size()));


    // Only calculate velocity from relevant grid nodes
    for (int a = -1; a < 3; ++a) {
        for (int b = -1; b < 3; ++b) {
            for (int c = -1; c < 3; ++c) {
                const int coord_i = x_index + a;
                const int coord_j = y_index + b;
                const int coord_k = z_index + c;
                const double weight_ip = particle.m_weights[GridCoordinate{ coord_i, coord_j, coord_k }];
                grid.GetVelocity(temp_velocity, coord_i, coord_j, coord_k);
                grid.GetOldVelocity(temp_old_velocity, coord_i, coord_j, coord_k);
                v_PIC += temp_velocity * weight_ip;
                v_FLIP += (temp_velocity - temp_old_velocity) * weight_ip;
            }
        }
    }
    particle.m_velocity =
        (1 - constants.flip_alpha) * v_PIC + constants.flip_alpha * v_FLIP;
  }
}
