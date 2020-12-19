#include "particle.hpp"

Particle::Particle() {
  m_velocity.setZero();
  m_def_elastic.setIdentity();
  m_def_plastic.setIdentity();
}

void Particle::resolve_collision(CollisionObject &collision_object, double dt) {
  collision_object.resolve_collision(m_position, m_velocity, dt);
}
