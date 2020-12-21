#pragma once

#include <Eigen/Core>

class Constants {
public:
  const double grid_cell_size = 0.1;
  const double hardening_coefficient = 10;
  const double critical_compression = 2.5e-2;
  //  const double critical_compression = 1.9e-2;
  const double critical_stretch = 7.5e-3;
  //  const double critical_stretch = 5.0e-3;

  const double flip_alpha = 0.95;

  const Eigen::Vector3d gravity = Eigen::Vector3d(0.0, -9.81, 0.0);

  const double youngs_modulus = 1.4e5;
  const double poissons_ratio = 0.2;

  // These must be after youngs_modulus and poissons_ratio, so they are
  // initialized after
  const double mu;
  const double lambda;

  Constants();
};
