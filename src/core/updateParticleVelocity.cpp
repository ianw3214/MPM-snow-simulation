#include "core/updateParticleVelocity.hpp"

void UpdateParticleVelocity(const Constants &constants,
                            std::vector<Particle> &particles,
                            const Grid &grid) {
  for (Particle &particle : particles) {
    Eigen::Vector3d v_PIC = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_FLIP = particle.m_velocity;
    for (const auto &node : grid) {
      const GridCoordinate &coords = node.first;
      const Grid::NodeData &data = node.second;
      const double weight_ip = particle.m_weights[coords];
      v_PIC += data.m_velocity * weight_ip;
      v_FLIP += (data.m_velocity - data.m_velocity_old) * weight_ip;
    }
    particle.m_velocity =
        (1 - constants.flip_alpha) * v_PIC + constants.flip_alpha * v_FLIP;
  }
}
