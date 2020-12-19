#include "collision_object.hpp"

void CollisionObject::resolve_collision(const Eigen::Vector3d &position,
                                        Eigen::Vector3d &velocity, double dt) {
  Eigen::Vector3d p2 = position + velocity;
  if (!detect_collision(p2))
    return;

  auto normal = get_normal(p2);
  auto v_n = normal.dot(velocity);
  if (v_n >= 0)
    return;

  auto v_t = velocity - v_n * normal;

  auto v_t_norm = v_t.norm();

  if (v_t_norm <= -friction_coeff * v_n) {
    velocity.setZero();
    return;
  }

  velocity = v_t + friction_coeff * v_n / v_t_norm * v_t;
}
