#ifndef FKwithlength_h_
#define FKwithlegnth_h_

#include <iostream>
#include <iomanip>
#include <array>
#include <cmath>

#include "matrixutils.hpp"

#define PI 3.14159265359

std::array<std::array<double, 4>, 4> forwardkinematics(std::array<double, 6> angles, std::array<double, 7> links);

#endif // FKwithlegnth_h_