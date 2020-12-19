#include "grid.hpp"

Grid::Grid(double granularity)
	: m_granularity(granularity)
{

}

void Grid::AppendMass(double x, double y, double z, double mass)
{
	auto it = m_nodes.find(Coordinate{ x, y, z });
	if (it == m_nodes.end())
	{
		m_nodes[{x, y, z}] = NodeData{ mass, Eigen::Vector3d() };
	}
	else
	{
		m_nodes[{x, y, z}].m_mass += mass;
	}
}

void Grid::AppendVelocity(double x, double y, double z, Eigen::Vector3d velocity)
{
	auto it = m_nodes.find(Coordinate{ x, y, z });
	if (it == m_nodes.end())
	{
		m_nodes[{x, y, z}] = NodeData{ 0.0, velocity };
	}
	else
	{
		m_nodes[{x, y, z}].m_velocity += velocity;
	}
}

double Grid::GetMass(double x, double y, double z)
{
	auto it = m_nodes.find(Coordinate{ x, y, z });
	if (it != m_nodes.end())
	{
		return m_nodes[{x, y, z}].m_mass;
	}
	return 0.0;
}