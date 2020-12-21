#include "camera.hpp"
#include <iostream>

Camera::Camera(const Eigen::Vector3d &position, const Eigen::Vector3d &forward,
               const Eigen::Vector3d &up, double fov_ratio)
    : position(position), forward(forward), up(up), dirty(true),
      fov_ratio(fov_ratio) {}

Eigen::Vector3d Camera::project(const Eigen::Vector3d &point) {
  if (dirty) {
    update_rotation_matrix();
    dirty = false;
  }
  Eigen::Vector3d result = rotation_matrix * (point - position);
  result(0) /= result(2) * fov_ratio;
  result(1) /= result(2) * fov_ratio;
  return result;
}

void Camera::update_rotation_matrix() {
  forward = forward.normalized();
  up = up.normalized();
  rotation_matrix.col(0) = up.cross(forward);
  rotation_matrix.col(1) = up;
  rotation_matrix.col(2) = forward;
  rotation_matrix = rotation_matrix.inverse().eval();
}

void Camera::set_position(const Eigen::Vector3d &value) {
  position = value;
  dirty = true;
}
