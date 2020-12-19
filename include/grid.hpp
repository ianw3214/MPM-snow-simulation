#pragma once

#include <map>

#include <Eigen/Core>

class Grid
{
	struct Coordinate
	{
		double m_x;
		double m_y;
		double m_z;

		// The coordinates need to be sortable in the map
		bool operator<(const Coordinate& other) const
		{
			return m_x == other.m_x ? m_y == other.m_y ? m_z < other.m_z : m_y < other.m_y : m_x < other.m_x;
		};
	};
	struct NodeData
	{
		double m_mass;
		Eigen::Vector3d m_velocity;
	};
public:
	Grid(double granularity = 0.1);

	inline double Granularity() const { return m_granularity; }

	void AppendMass(double x, double y, double z, double mass);
	void AppendVelocity(double x, double y, double z, Eigen::Vector3d velocity);

	double GetMass(double x, double y, double z);
private:
	std::map<Coordinate, NodeData> m_nodes;

	// grid properties
	double m_granularity;
};