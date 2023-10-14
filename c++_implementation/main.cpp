#include <iostream>
#include <array>
#include <cmath>

#include "includes/IKwithlength.hpp"
#include "includes/FKwithlength.hpp"

#define RD 57.29578
#define DR 0.0174533

int main() {

// euler angles and coordinates of the endeffector
std::array<std::array<double, 3>, 3> endrotation = eulertorotation(90 * DR, 0, 0);
std::array<double, 3> endposition = {200, 200, 200};

std::array<std::array<double, 4>, 4> endeffector2 = {{
    {endrotation[0][0], endrotation[0][1], endrotation[0][2], endposition[0]},
    {endrotation[1][0], endrotation[1][1], endrotation[1][2], endposition[1]},
    {endrotation[2][0], endrotation[2][1], endrotation[2][2], endposition[2]},
    {0, 0, 0, 1}
}};

//homogeneous transformation matrix of endeffector which respresents both position and orientation
std::array<std::array<double, 4>, 4> endeffector1 = {{
    {-0.9696, 0.1344, 0.2043, 293.2939},
    {0.2096, 0.8870, 0.4114, 224.6901},
    {-0.1259, 0.4418, -0.8883, 293.4121},
    {0, 0, 0, 1}
}};

// link of link lengths
std::array<double, 7> linklist = {187, 103, 270, 70, 134, 168, 72};

// compute joint angles
std::array<double, 6> IKresult1 = inversekinematics(endeffector1, linklist);
std::array<double, 6> IKresult2 = inversekinematics(endeffector2, linklist);


std::cout << IKresult1[0] << ", " << IKresult1[1] << ", " << IKresult1[2] << ", " << IKresult1[3] << ", " << IKresult1[4] << ", " << IKresult1[5] << std::endl;
std::cout << IKresult2[0] << ", " << IKresult2[1] << ", " << IKresult2[2] << ", " << IKresult2[3] << ", " << IKresult2[4] << ", " << IKresult2[5] << std::endl;

// compute end effector matrix
std::array<std::array<double, 4>, 4> FKresult1 = forwardkinematics(IKresult1, linklist);
std::array<std::array<double, 4>, 4> FKresult2 = forwardkinematics(IKresult2, linklist);

printmatrix4(FKresult1);
printmatrix4(FKresult2);

return 0;
}