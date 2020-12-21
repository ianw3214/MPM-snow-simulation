#include "constants.hpp"

#include <cstdio>

Constants::Constants()
    : mu(youngs_modulus / (2 * (1 + poissons_ratio))),
      lambda(youngs_modulus * poissons_ratio /
             ((1 + poissons_ratio) * (1 - 2 * poissons_ratio))) {}
