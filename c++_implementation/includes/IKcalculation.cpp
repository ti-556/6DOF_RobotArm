#include <iostream>
#include <iomanip>
#include <array>
#include <cmath>

#include "matrixutils.hpp"

#define PI 3.14159265359

//determining the wrist effector matrix from the end effector matrix
std::array<double, 4> wristcalculation(std::array<std::array<double, 4>, 4> endeffector) {

    std::array<double, 4> wristeffector;

    wristeffector[0] = endeffector[0][3] - 72 * endeffector[0][2];
    wristeffector[1] = endeffector[1][3] - 72 * endeffector[1][2];
    wristeffector[2] = endeffector[2][3] - 72 * endeffector[2][2];
    wristeffector[3] = endeffector[3][3] - 72 * endeffector[3][2];

    return wristeffector;
}

//calculating angle1 from the wrist effector matrix
std::array<double, 2> angle1calculation(std::array<double, 4> wristeffector) {

    std::array<double, 2> angle1result;

    angle1result[0] = atan(wristeffector[1] / wristeffector[0]);
    angle1result[1] = atan(wristeffector[1] / wristeffector[0]) + PI;

    return angle1result;
}

// calculating four possibilities of angle2 from angle1 and the wrist effector matrix
std::array<double, 4> angle2calculation(std::array<double, 4> wristeffector, std::array<double, 2> angle1) {

    std::array<double, 4> angle2result;

    // coefficients A, B, C from an equation with the following format: Acos(angle2) + Bsin(angle2) = C
    // coefficients using angle1A
    double A1 = -(-(wristeffector[2] - 290) * 270 * 2);
    double B1 = -(-wristeffector[0] / cos(angle1[0]) * 270 * 2);
    double C1 = -(pow(302, 2) + pow(70, 2) - pow(wristeffector[0] / cos(angle1[0]), 2) - pow(wristeffector[2] - 290, 2) - pow(270, 2));

    // coefficients using angle1B
    double A2 = -(-(wristeffector[2] - 290) * 270 * 2);
    double B2 = -(-wristeffector[0] / cos(angle1[1]) * 270 * 2);
    double C2 = -(pow(302, 2) + pow(70, 2) - pow(wristeffector[0] / cos(angle1[1]), 2) - pow(wristeffector[2] - 290, 2) - pow(270, 2));

    angle2result[0] = atan2(C1 , sqrt(pow(A1, 2) + pow(B1, 2) - pow(C1, 2))) - atan2(A1, B1);
    angle2result[1] = atan2(C1 , -sqrt(pow(A1, 2) + pow(B1, 2) - pow(C1, 2))) - atan2(A1, B1);
    angle2result[2] = atan2(C2 , sqrt(pow(A2, 2) + pow(B2, 2) - pow(C2, 2))) - atan2(A2, B2);
    angle2result[3] = atan2(C2 , -sqrt(pow(A2, 2) + pow(B2, 2) - pow(C2, 2))) - atan2(A2, B2);

    return angle2result;
}

// calculating four possibilities of angle3 from angle2, angle1, and the wrist effector matrix
std::array<double, 4> angle3calculation(std::array<double, 4> wristeffector, std::array<double, 4> angle2, std::array<double, 2> angle1) {

    std::array<double, 4> angle3result;

    // common coefficients A and B, and coefficients C for four possibilities of angle2
    double A = 302;
    double B = 70;
    double C1 = wristeffector[0]/cos(angle1[0]) - 270 * sin(angle2[0]);
    double C2 = wristeffector[0]/cos(angle1[0]) - 270 * sin(angle2[1]);
    double C3 = wristeffector[0]/cos(angle1[1]) - 270 * sin(angle2[2]);
    double C4 = wristeffector[0]/cos(angle1[1]) - 270 * sin(angle2[3]);

    angle3result[0] = atan2(C1 , -sqrt(pow(A, 2) + pow(B, 2) - pow(C1, 2))) - atan2(A, B) - angle2[0];
    angle3result[1] = atan2(C2 , -sqrt(pow(A, 2) + pow(B, 2) - pow(C2, 2))) - atan2(A, B) - angle2[1];
    angle3result[2] = atan2(C3 , -sqrt(pow(A, 2) + pow(B, 2) - pow(C3, 2))) - atan2(A, B) - angle2[2];
    angle3result[3] = atan2(C4 , -sqrt(pow(A, 2) + pow(B, 2) - pow(C4, 2))) - atan2(A, B) - angle2[3];

    return angle3result;
}

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
std::array<std::array<double, 3>, 3> Rotation3toE(std::array<std::array<double, 4>, 4> endeffector, double angle1, double angle2, double angle3) {

// taking out the rotation matrix from the homogeneous tranformation matrix of the endeffector
std::array<std::array<double, 3>, 3> RE = {{
    {endeffector[0][0], endeffector[0][1], endeffector[0][2]},
    {endeffector[1][0], endeffector[1][1], endeffector[1][2]},
    {endeffector[2][0], endeffector[2][1], endeffector[2][2]}
}};

// rotation matrices of joints 1, 2, and 3
std::array<std::array<double, 3>, 3> R1 = {{
    {cos(angle1), -sin(angle1), 0},
    {sin(angle1), cos(angle1), 0},
    {0, 0, 1}
}};

std::array<std::array<double, 3>, 3> R2 = {{
    {cos(angle2), 0, sin(angle2)},
    {0, 1, 0},
    {-sin(angle2), 0, cos(angle2)}
}};

std::array<std::array<double, 3>, 3> R3 = {{
    {cos(angle3), 0, sin(angle3)},
    {0, 1, 0},
    {-sin(angle3), 0, cos(angle3)}
}};

// adjustment rotation matrix
std::array<std::array<double, 3>, 3> R7 = {{
    {cos(PI / 2), 0, sin(PI / 2)},
    {0, 1, 0},
    {-sin(PI / 2), 0, cos(PI / 2)}
}};

// computing rotation matrix from joint 3 to end effector
std::array<std::array<double, 3>, 3> IR1 = MatInverse3(R1);
std::array<std::array<double, 3>, 3> IR2 = MatInverse3(R2);
std::array<std::array<double, 3>, 3> IR3 = MatInverse3(R3);
std::array<std::array<double, 3>, 3> IR7 = MatInverse3(R7);

std::array<std::array<double, 3>, 3> IR32 = MatMultiply3(IR3, IR2);
std::array<std::array<double, 3>, 3> IR31 = MatMultiply3(IR32, IR1);
std::array<std::array<double, 3>, 3> IR3E = MatMultiply3(IR31, RE);
std::array<std::array<double, 3>, 3> Rot3E = MatMultiply3(IR3E, IR7);

return Rot3E;
}

// calculating angle 4 from the rotation matrix from thrid joint to the end effector and two possibilities of angle 5
std::array<double, 2> angle4calculation(std::array<std::array<double, 3>, 3> Rot3E, std::array<double, 2> angle5) {

    std::array<double, 2> angle4result;

    angle4result[0] = atan2(Rot3E[1][0] / sin(angle5[0]), -Rot3E[2][0] / sin(angle5[0]));
    angle4result[1] = atan2(Rot3E[1][0] / sin(angle5[1]), -Rot3E[2][0] / sin(angle5[1]));

    return angle4result;
}

// calculating two possibilities of angle 5 from the rotation matrix from third joint to end effector determined based on angles 1, 2, 3
std::array<double, 2> angle5calculation(std::array<std::array<double, 3>, 3> Rot3E) {

    std::array<double, 2> angle5result;

    angle5result[0] = atan2(sqrt(pow(Rot3E[0][1], 2) + pow(Rot3E[0][2], 2)), Rot3E[0][0]);
    angle5result[1] = atan2(-sqrt(pow(Rot3E[0][1], 2) + pow(Rot3E[0][2], 2)), Rot3E[0][0]);

    return angle5result;
}

// calculating angle 6 from the rotation matrix from thrid joint to the end effector and two possibilities of angle 5
std::array<double, 2> angle6calculation(std::array<std::array<double, 3>, 3> Rot3E, std::array<double, 2> angle5) {

    std::array<double, 2> angle6result;

    angle6result[0] = atan2(Rot3E[0][1] / sin(angle5[0]), Rot3E[0][2] / sin(angle5[0]));
    angle6result[1] = atan2(Rot3E[0][1] / sin(angle5[1]), Rot3E[0][2] / sin(angle5[1]));

    return angle6result;
}