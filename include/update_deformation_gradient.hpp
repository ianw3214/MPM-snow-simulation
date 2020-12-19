#pragma once

#include "constants.hpp"
#include "grid.hpp"
#include "particle.hpp"
#include <Eigen/Dense>
#include <cmath>

/**
 * @tparam F Type of lambda
 * @param constants The simulation constants
 * @param particle The particle to update
 * @param grid The grid (assuming previous steps are done)
 * @param dt Delta time
 * @param get_grad_weight A lambda function of type:
 *     (int i, int j, int k, const Eigen::Vector3d &position) -> Eigen::Vector3d
 *     It should return gradient of grid weight given the grid cell index and
 *     particle position.
 */
template <typename F>
void update_deformation_gradient(const Constants &constants, Particle &particle,
                                 const Grid &grid, double dt,
                                 F get_grad_weight) {
  Eigen::Matrix3d grad_v;
  grad_v.setZero();
  Eigen::Vector3d tmp_velocity;
  auto coord = grid.get_coordinate(particle.m_position);
  for (int a = -1; a < 3; ++a) {
    for (int b = -1; b < 3; ++b) {
      for (int c = -1; c < 3; ++c) {
        int i = coord.i + a;
        int j = coord.j + b;
        int k = coord.k + c;
        Eigen::Vector3d grad_weight =
            get_grad_weight(i, j, k, particle.m_position);
        grid.GetVelocity(tmp_velocity, i, j, k);
        grad_v += tmp_velocity * grad_weight.transpose();
      }
    }
  }

  Eigen::Matrix3d def_elastic_next =
      (Eigen::Matrix3d::Identity() + dt * grad_v) * particle.m_def_elastic;
  Eigen::Matrix3d def_next = def_elastic_next * particle.m_def_plastic;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      def_elastic_next, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector3d sigma = svd.singularValues();

  double a = 1 - constants.critical_compression,
         b = 1 + constants.critical_stretch;
  sigma(0) = std::clamp(sigma(0), a, b);
  sigma(1) = std::clamp(sigma(1), a, b);
  sigma(2) = std::clamp(sigma(2), a, b);

  particle.m_def_elastic =
      svd.matrixU() * sigma.asDiagonal() * svd.matrixV().transpose();
  particle.m_def_plastic = svd.matrixV() * sigma.cwiseInverse().asDiagonal() *
                           svd.matrixU().transpose() * def_next;
}