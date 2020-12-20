#include "simulation.hpp"
#include <Eigen/Dense>
#include <ground.hpp>
#include <iostream>
#include <memory>
#include <chrono>

int main(int argc, char *argv[]) {
  Constants constants;
  Simulation simulation(constants);

  simulation.Init();
  simulation.add_collision_object(std::make_unique<Ground>(0));

  auto updateStart = std::chrono::system_clock::now();
  simulation.Update(0.016);
  double time_elapsed = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - updateStart).count());
  std::cout << "Time elapsed: " << time_elapsed << "us (" << time_elapsed / 1000000.0 << " s)" << std::endl;

  std::cout << "Simulation finished." << std::endl;

  return 0;
}
