#include "simulation.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <ground.hpp>
#include <iostream>
#include <memory>

#define NUM_FRAMES 100

int main(int argc, char *argv[]) {
  Constants constants;
  Simulation simulation(constants);

  simulation.add_sphere(0.5, 15, Eigen::Vector3d(-2, 2, 0),
                        Eigen::Vector3d(7, 0, 0), 0.2);
  simulation.add_sphere(0.5, 15, Eigen::Vector3d(2, 2, 0),
                        Eigen::Vector3d(-7, 0, 0), 0.2);
  //  simulation.add_sphere(2, 20, Eigen::Vector3d(-5, 4, 0),
  //                        Eigen::Vector3d(7, 0, 0), 0.2);
  //  simulation.add_sphere(2, 20, Eigen::Vector3d(5, 4, 0),
  //                        Eigen::Vector3d(-7, 0, 0), 0.2);
  simulation.add_collision_object(std::make_unique<Ground>(0));

  simulation.camera.set_position(Eigen::Vector3d(0, 2, -8));

  for (int i = 0; i < NUM_FRAMES; ++i) {
    auto updateStart = std::chrono::system_clock::now();
    simulation.Update(0.016);
    double time_elapsed = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - updateStart)
            .count());
    std::cout << "Frame " << i << ": Time elapsed: " << time_elapsed << "us ("
              << time_elapsed / 1000000.0 << " s)" << std::endl;
  }

  std::cout << "Simulation finished." << std::endl;

  return 0;
}
