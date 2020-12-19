#pragma once

#include <unordered_map>

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
  };

  explicit Grid(double cell_size = 0.1);

  inline double cell_size() const { return m_cell_size; }

  void AppendMass(int x, int y, int z, double mass);
  void AppendVelocity(int x, int y, int z, const Eigen::Vector3d &velocity);

  double GetMass(int x, int y, int z);

  void clear();

private:
  std::unordered_map<GridCoordinate, NodeData> m_nodes;

  // grid properties
  double m_cell_size;
};
