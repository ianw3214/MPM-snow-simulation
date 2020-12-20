#include "grid.hpp"

Grid::Grid(double cell_size) : m_cell_size(cell_size) {}

GridCoordinate Grid::get_coordinate(const Eigen::Vector3d &position) const {
  return {static_cast<int>(std::floor(position(0) / m_cell_size)),
          static_cast<int>(std::floor(position(1) / m_cell_size)),
          static_cast<int>(std::floor(position(2) / m_cell_size))};
}

void Grid::AppendMass(int x, int y, int z, double mass) {
  auto it = m_nodes.find(GridCoordinate{x, y, z});
  if (it == m_nodes.end()) {
    m_nodes[{x, y, z}] = NodeData{mass, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  } else {
    m_nodes[{x, y, z}].m_mass += mass;
  }
}

void Grid::AppendVelocity(int x, int y, int z,
                          const Eigen::Vector3d &velocity) {
  auto it = m_nodes.find(GridCoordinate{x, y, z});
  if (it == m_nodes.end()) {
    m_nodes[{x, y, z}] = NodeData{0.0, velocity, Eigen::Vector3d::Zero()};
  } else {
    m_nodes[{x, y, z}].m_velocity += velocity;
  }
}

void Grid::AppendForce(int x, int y, int z, const Eigen::Vector3d& force) {
  auto it = m_nodes.find(GridCoordinate{ x, y, z });
  if (it == m_nodes.end()) {
      m_nodes[{x, y, z}] = NodeData{ 0.0, Eigen::Vector3d::Zero(), force};
  }
  else {
      m_nodes[{x, y, z}].m_force += force;
  }
}

double Grid::GetMass(int x, int y, int z) const {
  auto it = m_nodes.find(GridCoordinate{x, y, z});
  if (it != m_nodes.end()) {
    return (*it).second.m_mass;
  }
  return 0.0;
}

void Grid::GetVelocity(Eigen::Vector3d &out, int x, int y, int z) const {
  auto it = m_nodes.find(GridCoordinate{x, y, z});
  if (it != m_nodes.end()) {
    out = (*it).second.m_velocity;
  }
  out.setZero();
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
