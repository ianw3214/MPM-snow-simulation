#include "core/updateGridVelocity.hpp"

void UpdateGridVelocity(Grid& grid, double dt)
{
  for (auto& node : grid)
  {
	Grid::NodeData& data = node.second;
	Eigen::Vector3d old_velocity = data.m_velocity;
	if (data.m_mass == 0.0) {
	  data.m_velocity = Eigen::Vector3d::Zero();
	}
	else {
	  data.m_velocity = data.m_velocity + dt * data.m_force / data.m_mass;
	}
	data.m_velocity_old = old_velocity;
  }
}