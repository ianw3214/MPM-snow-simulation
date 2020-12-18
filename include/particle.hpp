#pragma once

#include <Eigen/Core>

class Particle
{
public:
    Eigen::Vector3d m_position;
    Eigen::Vector3d m_velocity;

    float m_mass;
    Eigen::Matrix3d m_deformation;
private:
};