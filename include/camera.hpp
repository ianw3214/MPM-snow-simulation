#pragma once

#include <Eigen/Dense>

class Camera {
public:
  Eigen::Vector3d position, forward, up;
};
