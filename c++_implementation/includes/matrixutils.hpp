#ifndef matrixutils_h_ 
#define matrixutils_h_

#include <array>

std::array<std::array<double, 3>, 3> MatMultiply3(std::array<std::array<double, 3>, 3> matA, std::array<std::array<double, 3>, 3> matB);
std::array<std::array<double, 4>, 4> MatMultiply4(std::array<std::array<double, 4>, 4> matA, std::array<std::array<double, 4>, 4> matB);

std::array<std::array<double, 3>, 3> MatInverse3(std::array<std::array<double, 3>, 3> mat);

double closeto(double a, double b);

void printmatrix3(std::array<std::array<double, 3>, 3> mat);
void printmatrix4(std::array<std::array<double, 4>, 4> mat);

std::array<std::array<double, 3>, 3> eulertorotation(double roll, double pitch, double yaw);
std::array<double, 6> rotationtoeuler(std::array<std::array<double, 3>, 3> rotmatrix);

#endif /* matrixutils_h_ */
