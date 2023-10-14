#ifndef IKwithlength_h_
#define IKwithlength_h_

#include <iostream>
#include <iomanip>
#include <array>
#include <cmath>

#include "matrixutils.hpp"

#define PI 3.14159265359

std::array<double, 4> wristcalculation(std::array<std::array<double, 4>, 4> endeffector, std::array<double, 7> link);

std::array<double, 2> angle1calculation(std::array<double, 4> wristeffector);
std::array<double, 4> angle2calculation(std::array<double, 4> wristeffector, std::array<double, 2> angle1, std::array<double, 7> link);
std::array<double, 4> angle3calculation(std::array<double, 4> wristeffector, std::array<double, 4> angle2, std::array<double, 2> angle1, std::array<double, 7> link);


std::array<std::array<double, 3>, 3> Rotation3toE(std::array<std::array<double, 4>, 4> endeffector, double angle1, double angle2, double angle3);


std::array<double, 2> angle4calculation(std::array<std::array<double, 3>, 3> Rot3E, std::array<double, 2> angle5);
std::array<double, 2> angle5calculation(std::array<std::array<double, 3>, 3> Rot3E);
std::array<double, 2> angle6calculation(std::array<std::array<double, 3>, 3> Rot3E, std::array<double, 2> angle5);

std::array<double, 6> inversekinematics(std::array<std::array<double, 4>, 4> endeff, std::array<double, 7> link);

#endif /*IKwithlength_h_*/