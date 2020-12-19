#include "particle.hpp"

void Particle::resolve_collision(CollisionObject &collision_object, double dt) {
  collision_object.resolve_collision(m_position, m_velocity, dt);
}
