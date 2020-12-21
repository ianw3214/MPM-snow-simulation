#pragma once

#include <Eigen/Dense>

class Camera {
private:
  Eigen::Vector3d position, forward, up;
  bool dirty;
  double fov_ratio;
  Eigen::Matrix3d rotation_matrix;

  void update_rotation_matrix();

public:
  Camera(const Eigen::Vector3d &position, const Eigen::Vector3d &forward,
         const Eigen::Vector3d &up, double fov_ratio);

  Eigen::Vector3d project(const Eigen::Vector3d &point);
  void set_position(const Eigen::Vector3d &value);
};
