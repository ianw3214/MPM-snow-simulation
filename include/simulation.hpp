#pragma once

#include <Eigen/Core>

#include "camera.hpp"
#include "constants.hpp"
#include "grid.hpp"
#include "image.hpp"
#include "particle.hpp"
#include <memory>
#include <random>
#include <vector>

class Simulation {
public:
  explicit Simulation(const Constants &constants);

  void add_sphere(double radius, unsigned int edge_length,
                  const Eigen::Vector3d &position,
                  const Eigen::Vector3d &velocity, double mass);
  void Update(double dt);

  const Constants constants;

  std::vector<Particle> particles;
  std::vector<std::unique_ptr<CollisionObject>> collision_objects;
  Grid grid;

  Camera camera;
  Image image;
  double render_depth_near = 1;
  double render_depth_far = 10;

  void add_collision_object(std::unique_ptr<CollisionObject> ptr);

  void render(int frame_index);

private:
  bool m_firstTick;
  std::mt19937 gen;
};
