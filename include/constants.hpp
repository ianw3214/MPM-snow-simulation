#pragma once

#include <Eigen/Core>

class Constants {
public:
  const double hardening_coefficient = 10;
  const double critical_compression = 2.5e-2;
  const double critical_stretch = 7.5e-3;

  const double mu = 0.5;
  const double lambda = 0.5;

  const Eigen::Vector3d gravity = Eigen::Vector3d(0.0, -9.81, 0.0);
};
