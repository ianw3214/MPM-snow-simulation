#include "simulation.hpp"
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char *argv[]) {
  Constants constants;
  Simulation simulation(constants);

  simulation.Update(0.016);

  std::cout << "Simulation finished." << std::endl;

  return 0;
}
