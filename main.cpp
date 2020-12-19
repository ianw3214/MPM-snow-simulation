#include "simulation.hpp"
#include <Eigen/Dense>
#include <ground.hpp>
#include <iostream>
#include <memory>

int main(int argc, char *argv[]) {
  Constants constants;
  Simulation simulation(constants);

  simulation.add_collision_object(std::make_unique<Ground>(0));

  simulation.Update(0.016);

  std::cout << "Simulation finished." << std::endl;

  return 0;
}
