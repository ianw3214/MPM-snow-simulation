#include "grid.hpp"

Grid::Grid(double cell_size) : m_cell_size(cell_size) {}

void Grid::AppendMass(int x, int y, int z, double mass) {
  auto it = m_nodes.find(GridCoordinate{x, y, z});
  if (it == m_nodes.end()) {
    m_nodes[{x, y, z}] = NodeData{mass, Eigen::Vector3d()};
  } else {
    m_nodes[{x, y, z}].m_mass += mass;
  }
}

void Grid::AppendVelocity(int x, int y, int z,
                          const Eigen::Vector3d &velocity) {
  auto it = m_nodes.find(GridCoordinate{x, y, z});
  if (it == m_nodes.end()) {
    m_nodes[{x, y, z}] = NodeData{0.0, velocity};
  } else {
    m_nodes[{x, y, z}].m_velocity += velocity;
  }
}

double Grid::GetMass(int x, int y, int z) {
  auto it = m_nodes.find(GridCoordinate{x, y, z});
  if (it != m_nodes.end()) {
    return m_nodes[{x, y, z}].m_mass;
  }
  return 0.0;
}

void Grid::resolve_collision(CollisionObject &collision_object, double dt) {
  for (auto &entry : m_nodes) {
    Eigen::Vector3d position(entry.first.i * m_cell_size,
                             entry.first.j * m_cell_size,
                             entry.first.k * m_cell_size);
    collision_object.resolve_collision(position, entry.second.m_velocity, dt);
  }
}

void Grid::clear() { m_nodes.clear(); }

bool GridCoordinate::operator==(const GridCoordinate &other) const {
  return i == other.i && j == other.j && k == other.k;
}
