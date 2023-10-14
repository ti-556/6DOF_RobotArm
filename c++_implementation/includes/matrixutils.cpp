/*
This is an utility header file for basic matrix manipulations
- multiplicaiton
- inversion
- euler to rotation matrix conversion
- printing
*/

#include <array>
#include <iostream>
#include <cmath>

#include "matrixutils.hpp"

#define PI 3.14159265359

std::array<std::array<double, 3>, 3> MatMultiply3(std::array<std::array<double, 3>, 3> matA, std::array<std::array<double, 3>, 3> matB) {

    std::array<std::array<double, 3>, 3> matC;

    for (int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            matC[i][j] = 0;
            for(int k = 0; k < 3; k++){
                matC[i][j] = matC[i][j] + matA[i][k] * matB[k][j];
            }
        }
}

return matC;
}

std::array<std::array<double, 4>, 4> MatMultiply4(std::array<std::array<double, 4>, 4> matA, std::array<std::array<double, 4>, 4> matB) {

    std::array<std::array<double, 4>, 4> matC;

    for (int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            matC[i][j] = 0;
            for(int k = 0; k < 4; k++){
                matC[i][j] = matC[i][j] + matA[i][k] * matB[k][j];
            }
        }
    }

    return matC;
}

std::array<std::array<double, 3>, 3> MatInverse3(std::array<std::array<double, 3>, 3> mat) {

    std::array<std::array<double, 3>, 3> matC;

    float det = 0;

    // determinant calculation
    for(int i = 0; i < 3; i++) {
        det = det + (mat[0][i] * (mat[1][(i + 1) % 3] * mat[2][(i + 2) % 3] - mat[1][(i + 2) % 3] * mat[2][(i + 1) % 3]));
    }

    // inverse of a matrix exists if it's determinant is non-zero
    if(det != 0) {
        // each entry of the resulting matrix is the cofactor of the original matrix divided by it's determinant
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                matC[i][j] = (mat[(j + 1) % 3][(i + 1) % 3] * mat[(j + 2) % 3][(i + 2) % 3] - mat[(j + 1) % 3][(i + 2) % 3] * mat[(j + 2) % 3][(i + 1) % 3]) / det;
            }
        }
    }
    else {
        std::cout << "no inverse for this matrix." << std::endl;
        matC = {{
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
        }};
    }

    return matC;
}

double closeto(double a, double b) {
    if (fabs(a - b) < 0.00001) {
        return a;
    }
    else {
        return b;
    }
}

void printmatrix3(std::array<std::array<double, 3>, 3> mat) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            std::cout << closeto(0, mat[i][j]) << "   ";
        }
        std::cout << "\n";
    }
}

void printmatrix4(std::array<std::array<double, 4>, 4> mat) {
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            std::cout << closeto(0, mat[i][j]) << "   ";
        }
        std::cout << "\n";
    }
}

// conversion from euler angles (roll, pitch, and yaw) to rotation matrix
std::array<std::array<double, 3>, 3> eulertorotation(double roll, double pitch, double yaw) {
    std::array<std::array<double, 3>, 3> rollrot = {{
        {1, 0, 0},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll), cos(roll)}
    }};
    std::array<std::array<double, 3>, 3> pitchrot = {{
        {cos(pitch), 0, sin(pitch)},
        {0, 1, 0},
        {-sin(pitch), 0, cos(pitch)}
    }};
    std::array<std::array<double, 3>, 3> yawrot = {{
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}
    }};

    std::array<std::array<double, 3>, 3> rotmatrix = MatMultiply3(yawrot, MatMultiply3(pitchrot, rollrot));

    return rotmatrix;
}

// conversion from rotation matrix to euler angles (roll, pitch, yaw)
std::array<double, 6> rotationtoeuler(std::array<std::array<double, 3>, 3> rotmatrix) {

    std::array<double, 6> eulerangles;
    if (rotmatrix[2][0] == -1 || rotmatrix[2][0] == 1) {
        // pitch
        eulerangles[2] = -asin(rotmatrix[2][0]);
        eulerangles[3] = PI - eulerangles[2];

        // roll
        eulerangles[0] = atan2(rotmatrix[2][1] / cos(eulerangles[2]), rotmatrix[2][2] / cos(eulerangles[2]));
        eulerangles[1] = atan2(rotmatrix[2][1] / cos(eulerangles[3]), rotmatrix[2][2] / cos(eulerangles[3]));

        // yaw
        eulerangles[4] = atan2(rotmatrix[1][0] / cos(eulerangles[2]), rotmatrix[0][0] / cos(eulerangles[2]));
        eulerangles[5] = atan2(rotmatrix[1][0] / cos(eulerangles[3]), rotmatrix[0][0] / cos(eulerangles[3]));
    }
    else {
        // yaw
        eulerangles[4] = 0;
        eulerangles[5] = 0;
        if (rotmatrix[2][0] == -1) {
            eulerangles[2] = PI / 2;
            eulerangles[3] = eulerangles[2];
            eulerangles[0] = eulerangles[4] + atan2(rotmatrix[0][1], rotmatrix[0][2]);
            eulerangles[1] = eulerangles[0];
        }
        else {
            eulerangles[2] = -PI / 2;
            eulerangles[3] = eulerangles[2];
            eulerangles[0] = -eulerangles[4] + atan2(-rotmatrix[0][1], -rotmatrix[0][2]);
            eulerangles[1] = eulerangles[0];
        }

    }

    return eulerangles;
}