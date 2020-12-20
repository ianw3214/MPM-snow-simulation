#pragma once

#include <unordered_map>

#include "collision_object.hpp"
#include <Eigen/Core>

class GridCoordinate {
public:
  int i;
  int j;
  int k;

  bool operator==(const GridCoordinate &other) const;
};

// Make coordinate hashable
namespace std {
template <> struct hash<GridCoordinate> {
  std::size_t operator()(const GridCoordinate &k) const {
    using std::hash;
    using std::size_t;
    return ((hash<int>()(k.i) ^ (hash<int>()(k.j) << 1)) >> 1) ^
           (hash<int>()(k.k) << 1);
  }
};

} // namespace std
class Grid {
public:
  struct NodeData {
    double m_mass;
    Eigen::Vector3d m_velocity;
    Eigen::Vector3d m_force;
  };

  explicit Grid(double cell_size = 0.1);

  inline double cell_size() const { return m_cell_size; }

  GridCoordinate get_coordinate(const Eigen::Vector3d &position) const;

  void AppendMass(int x, int y, int z, double mass);
  void AppendVelocity(int x, int y, int z, const Eigen::Vector3d &velocity);

  double GetMass(int x, int y, int z) const;
  void GetVelocity(Eigen::Vector3d &out, int x, int y, int z) const;

  void resolve_collision(CollisionObject &collision_object, double dt);

  void clear();

  std::unordered_map<GridCoordinate, NodeData>::iterator begin() { return m_nodes.begin(); }
  std::unordered_map<GridCoordinate, NodeData>::const_iterator begin() const { return m_nodes.begin(); }
  std::unordered_map<GridCoordinate, NodeData>::iterator end() { return m_nodes.end(); }
  std::unordered_map<GridCoordinate, NodeData>::const_iterator end() const { return m_nodes.end(); }

private:
  std::unordered_map<GridCoordinate, NodeData> m_nodes;

  // grid properties
  double m_cell_size;
};
