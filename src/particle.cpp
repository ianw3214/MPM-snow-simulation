#include "particle.hpp"

Particle::Particle() 
	: m_def_elastic_det(1.0)
	, m_def_plastic_det(1.0)
{
  m_velocity.setZero();
  m_def_elastic.setIdentity();
  m_def_plastic.setIdentity();
  m_weights.reserve(4 * 4 * 4);
  m_weight_derivatives.reserve(4 * 4 * 4);
}

Particle::Particle(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double m, double vol, double d)
	: m_position(p)
	, m_velocity(v)
	, m_mass(m)
	, m_volume(vol)
	, m_density(d)
	, m_def_elastic_det(1.0)
	, m_def_plastic_det(1.0)
{
	m_def_elastic.setIdentity();
	m_def_plastic.setIdentity();
	m_weights.reserve(4 * 4 * 4);
	m_weight_derivatives.reserve(4 * 4 * 4);
}

void Particle::resolve_collision(CollisionObject &collision_object, double dt) {
  collision_object.resolve_collision(m_position, m_velocity, dt);
}
