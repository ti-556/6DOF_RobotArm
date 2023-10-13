#ifndef IKcalculation_h_
#define IKcalculation_h_

#include <iostream>
#include <iomanip>
#include <array>
#include <cmath>

#include "matrixutils.hpp"

#define PI 3.14159265359

//determining the wrist effector matrix from the end effector matrix
std::array<double, 4> wristcalculation(std::array<std::array<double, 4>, 4> endeffector);

//calculating angle1 from the wrist effector matrix
std::array<double, 2> angle1calculation(std::array<double, 4> wristeffector);

// calculating four possibilities of angle2 from angle1 and the wrist effector matrix
std::array<double, 4> angle2calculation(std::array<double, 4> wristeffector, std::array<double, 2> angle1);

// calculating four possibilities of angle3 from angle2, angle1, and the wrist effector matrix
std::array<double, 4> angle3calculation(std::array<double, 4> wristeffector, std::array<double, 4> angle2, std::array<double, 2> angle1);

/*
Determining the rotation matrix joint 3 to the end effector using given matrix of end effector and angles 1, 2, 3
The manually computed rotation matrix (in terms of angles 4, 5, 6) is:
{{
    {cos(theta5),                   sin(theta5) * sin(theta6),                                              cos(theta6) * sin(theta5)}
    {sin(theta4) * sin(theta5),     cos(theta4) * cos(theta6) - cos(theta5) * cos(theta4) * sin(theta6),    -cos(theta4) * sin(theta6) - cos(theta5) * cos(theta6) * sin(theta4)}
    {-cos(theta4) * sin(theta5),    cos(theta6) * sin(theta4) + cos(theta4) * cos(theta5) * sin(theta6),    cos(theta4) * cos(theta5) * cos(theta6) - sin(theta4) * sin(theta6)}
}}
So angles 4, 5, and 6 can be determined by the manual and computed rotation matrix
*/
std::array<std::array<double, 3>, 3> Rotation3toE(std::array<std::array<double, 4>, 4> endeffector, double angle1, double angle2, double angle3);

// calculating angle 4 from the rotation matrix from thrid joint to the end effector and two possibilities of angle 5
std::array<double, 2> angle4calculation(std::array<std::array<double, 3>, 3> Rot3E, std::array<double, 2> angle5);

// calculating two possibilities of angle 5 from the rotation matrix from third joint to end effector determined based on angles 1, 2, 3
std::array<double, 2> angle5calculation(std::array<std::array<double, 3>, 3> Rot3E);
// calculating angle 6 from the rotation matrix from thrid joint to the end effector and two possibilities of angle 5
std::array<double, 2> angle6calculation(std::array<std::array<double, 3>, 3> Rot3E, std::array<double, 2> angle5);

#endif /*IKcalculation_h_*/