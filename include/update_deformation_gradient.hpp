#pragma once

#include "constants.hpp"
#include "grid.hpp"
#include "particle.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

/**
 * @tparam F Type of lambda
 * @param constants The simulation constants
 * @param particle The particle to update
 * @param grid The grid (assuming previous steps are done)
 * @param dt Delta time
 * @param get_grad_weight A lambda function of type: (const GridCoordinate
 *     &coord, const Particle &particle) -> Eigen::Vector3d. It should return
 *     gradient of grid weight given the grid cell index and particle position.
 */
template <typename F>
void update_deformation_gradient(const Constants &constants, Particle &particle,
                                 const Grid &grid, double dt, F get_grad_weight,
                                 bool debug) {
  Eigen::Matrix3d grad_v;
  grad_v.setZero();
  Eigen::Vector3d tmp_velocity;
  Eigen::Vector3d grad_weight;
  auto coord = grid.get_coordinate(particle.m_position);
  auto temp_coord = coord;
  for (int a = -1; a < 3; ++a) {
    for (int b = -1; b < 3; ++b) {
      for (int c = -1; c < 3; ++c) {
        int i = coord.i + a;
        int j = coord.j + b;
        int k = coord.k + c;
        temp_coord.i = i;
        temp_coord.j = j;
        temp_coord.k = k;
        grad_weight = get_grad_weight(temp_coord, particle);
        grid.GetVelocity(tmp_velocity, i, j, k);
        grad_v += tmp_velocity * grad_weight.transpose();
      }
    }
  }
  if (debug) {
    Eigen::Vector3d grad_weight_sum = Eigen::Vector3d::Zero();
    for (int a = -1; a < 3; ++a) {
      for (int b = -1; b < 3; ++b) {
        for (int c = -1; c < 3; ++c) {
          int i = coord.i + a;
          int j = coord.j + b;
          int k = coord.k + c;
          temp_coord.i = i;
          temp_coord.j = j;
          temp_coord.k = k;
          grad_weight = get_grad_weight(temp_coord, particle);
          grad_weight_sum += grad_weight;
        }
      }
    }
    std::cout << grad_weight_sum << std::endl;
    std::cout << grad_weight << std::endl;
    std::cout << tmp_velocity << std::endl;
    std::cout << grad_v << std::endl;
  }

  Eigen::Matrix3d def_elastic_next =
      (Eigen::Matrix3d::Identity() + dt * grad_v) * particle.m_def_elastic;
  Eigen::Matrix3d def_next = def_elastic_next * particle.m_def_plastic;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      def_elastic_next, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector3d sigma = svd.singularValues();

  if (debug) {
    std::cout << sigma << std::endl;
  }

  double a = 1 - constants.critical_compression,
         b = 1 + constants.critical_stretch;
  sigma(0) = std::clamp(sigma(0), a, b);
  sigma(1) = std::clamp(sigma(1), a, b);
  sigma(2) = std::clamp(sigma(2), a, b);

  particle.m_def_elastic =
      svd.matrixU() * sigma.asDiagonal() * svd.matrixV().transpose();
  particle.m_def_plastic = svd.matrixV() * sigma.cwiseInverse().asDiagonal() *
                           svd.matrixU().transpose() * def_next;
  particle.m_def_elastic_det = particle.m_def_elastic.determinant();
  particle.m_def_plastic_det = particle.m_def_plastic.determinant();
  if (debug) {
    std::cout << particle.m_def_plastic_det << std::endl;
    std::cout << "====" << std::endl;
  }
}
