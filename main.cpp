#include "simulation.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <ground.hpp>
#include <iostream>
#include <memory>

#define FRAME_RATE 60
#define NUM_FRAMES 300
#define NUM_STEPS_PER_FRAME 20

namespace {
/**
 * This one needs NUM_STEPS_PER_FRAME to be 20
 */
void small_snowball_smash(Simulation &simulation) {
  simulation.add_sphere(0.5, 10, Eigen::Vector3d(-2, 2, 0),
                        Eigen::Vector3d(12, 0, 0), 0.2);
  simulation.add_sphere(0.5, 10, Eigen::Vector3d(2, 2, 0),
                        Eigen::Vector3d(-12, 0, 0), 0.2);
  simulation.add_collision_object(std::make_unique<Ground>(0));
  simulation.camera.set_position(Eigen::Vector3d(0, 1, -4));
}

void big_snowball_smash(Simulation &simulation) {
  simulation.add_sphere(2, 20, Eigen::Vector3d(-5, 4, 0),
                        Eigen::Vector3d(7, 0, 0), 0.2);
  simulation.add_sphere(2, 20, Eigen::Vector3d(5, 4, 0),
                        Eigen::Vector3d(-7, 0, 0), 0.2);
  simulation.add_collision_object(std::make_unique<Ground>(0));
  simulation.camera.set_position(Eigen::Vector3d(0, 2, -8));
}

void snowball_on_ground(Simulation &simulation) {
  simulation.add_sphere(1, 20, Eigen::Vector3d(0, 1, 0),
                        Eigen::Vector3d(0, 0, 0), 0.2);
  simulation.add_collision_object(std::make_unique<Ground>(0));
  simulation.camera.set_position(Eigen::Vector3d(0, 1, -4));
  simulation.render_depth_near = 3;
  simulation.render_depth_far = 5.5;
}

/**
 * This one needs NUM_STEPS_PER_FRAME to be 20
 */
void two_snowball(Simulation &simulation) {
  simulation.add_sphere(0.5, 10, Eigen::Vector3d(0, 0.5, 0),
                        Eigen::Vector3d(0, 0, 0), 0.2);
  simulation.add_sphere(0.5, 10, Eigen::Vector3d(0, 2, 0),
                        Eigen::Vector3d(0, 0, 0), 0.2);
  simulation.add_collision_object(std::make_unique<Ground>(0));
  simulation.camera.set_position(Eigen::Vector3d(0, 1, -4));
  simulation.render_depth_near = 3;
  simulation.render_depth_far = 5.5;
}

/**
 * This one needs NUM_STEPS_PER_FRAME to be 20
 */
void snowball_drop(Simulation &simulation) {
  simulation.add_sphere(1, 15, Eigen::Vector3d(-4, 4, 0),
                        Eigen::Vector3d(5, 1, 0), 0.2);
  simulation.add_collision_object(std::make_unique<Ground>(0));
  simulation.camera.set_position(Eigen::Vector3d(0, 1, -6));
  simulation.render_depth_near = 4.5;
  simulation.render_depth_far = 7.5;
}
} // namespace

int main(int argc, char *argv[]) {
  Constants constants;
  Simulation simulation(constants);

  //  small_snowball_smash(simulation);
  //  big_snowball_smash(simulation);
  //  snowball_on_ground(simulation);
  //  two_snowball(simulation);
  snowball_drop(simulation);

  for (int i = 0; i < NUM_FRAMES; ++i) {
    std::cout << "Frame " << i << ":" << std::endl;
    for (int j = 0; j < NUM_STEPS_PER_FRAME; ++j) {
      auto updateStart = std::chrono::system_clock::now();
      simulation.Update(1.0 / ((double)FRAME_RATE) /
                        ((double)NUM_STEPS_PER_FRAME));
      double time_elapsed = static_cast<double>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now() - updateStart)
              .count());
      std::cout << ": Time elapsed: " << time_elapsed << "us ("
                << time_elapsed / 1000000.0 << " s)" << std::endl;
    }
    simulation.render(i);
  }

  std::cout << "Simulation finished." << std::endl;

  return 0;
}
