#include "core/calculateForces.hpp"

// This is copied directly from rasterizeParticles
// TODO: can consider moving it to common header file
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
      return std::pow(x, 2) * 3.0 / 2.0 - 2.0 * x;
    else if (x_abs < 2)
      return -std::pow(x, 2) / 2.0 + 2.0 * x - 2.0;
    else
      return 0.0;
  }
  
  double calculate_weight(double h_inverse, double x_offset, double y_offset,
    double z_offset) {
    return N(h_inverse * x_offset) * N(h_inverse * y_offset) *
      N(h_inverse * z_offset);
  };

  double calculate_derivative(double h_inverse, double d_offset, double offset1, double offset2) {
    return dN(h_inverse * d_offset) * N(h_inverse * offset1) * N(h_inverse * offset2) * h_inverse;
  }
} // namespace

void CalculateForces(const Constants& constants, std::vector<Particle>& p, Grid& grid, double dt)
{
  // Common numbers used in calculations
  const double h_inverse = 1.0 / grid.cell_size();

  // Particle is not const here because we cache the weight derivatives in the particle itself
  for (Particle& particle : p)
  {
    // TODO: cache determinants to avoid re-calculating every time
    const double volume = particle.m_def_plastic_det * particle.m_volume;
    const double particle_x = particle.m_position(0);
    const double particle_y = particle.m_position(1);
    const double particle_z = particle.m_position(2);
    const int x_index =
        static_cast<int>(std::floor(particle_x / grid.cell_size()));
    const int y_index =
        static_cast<int>(std::floor(particle_y / grid.cell_size()));
    const int z_index =
        static_cast<int>(std::floor(particle_z / grid.cell_size()));
    // Need to first calculate the elastic deformation based on deformed location of grid node
    Eigen::Matrix3d accum = Eigen::Matrix3d::Identity();
    for (int a = -1; a < 3; ++a) {
      for (int b = -1; b < 3; ++b) {
        for (int c = -1; c < 3; ++c) {
          const int coord_i = x_index + a;
          const int coord_j = y_index + b;
          const int coord_k = z_index + c;

          Eigen::Vector3d d_w = particle.m_weight_derivatives[GridCoordinate{ coord_i, coord_j, coord_k }];
          Eigen::Vector3d v;
          grid.GetVelocity(v, coord_i, coord_j, coord_k);
          accum += dt * v * d_w.transpose();
        }
      }
    }
    particle.m_def_elastic_deformed = accum * particle.m_def_elastic;
    particle.m_def_elastic_deformed_det = particle.m_def_elastic_deformed.determinant();
    // Only calculate forces on relevant grid nodes
    for (int a = -1; a < 3; ++a) {
      for (int b = -1; b < 3; ++b) {
        for (int c = -1; c < 3; ++c) {
          const int coord_i = x_index + a;
          const int coord_j = y_index + b;
          const int coord_k = z_index + c;
          
          Eigen::Vector3d d_w = particle.m_weight_derivatives[GridCoordinate{coord_i, coord_j, coord_k}];
          // Calculate the cauchy stress
          const double JE = particle.m_def_elastic_deformed_det;
          const double JP = particle.m_def_plastic_det;
          Eigen::JacobiSVD<Eigen::Matrix3d> svd(particle.m_def_elastic, Eigen::ComputeFullU | Eigen::ComputeFullV);
          const Eigen::Matrix3d& U = svd.matrixU();
          const Eigen::Matrix3d& V = svd.matrixV();
          const Eigen::Matrix3d& S = svd.singularValues().asDiagonal();
          const Eigen::Matrix3d RE = U * V.transpose();
          const Eigen::Matrix3d SE = V * S * V.transpose();
          const double mu = constants.mu * std::exp(constants.hardening_coefficient * (1.0 - JP));
          const double lambda = constants.lambda * std::exp(constants.hardening_coefficient * (1.0 - JP));
          const Eigen::Matrix3d sigma = 2.0 * mu / JE * (particle.m_def_elastic - RE) * particle.m_def_elastic.transpose() + lambda / JE * (JE - 1.0) * JE * Eigen::Matrix3d::Identity();
          // Finally, add the sum to the forces
          grid.AppendForce(coord_i, coord_j, coord_k, volume* sigma* d_w);
        }
      }
    }
  }

  // Add gravity forces
  for (auto& node : grid) {
    Grid::NodeData& data = node.second;
    data.m_force += data.m_mass * constants.gravity;
  }
}