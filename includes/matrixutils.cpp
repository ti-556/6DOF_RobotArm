/*
This is an utility header file for basic matrix manipulations
- multiplicaiton
- inversion
- printing
*/

#include <array>
#include <iostream>

#include "matrixutils.hpp"

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

void printmatrix3(std::array<std::array<double, 3>, 3> mat) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            std::cout << mat[i][j] << "   ";
        }
        std::cout << "\n";
    }
}

void printmatrix4(std::array<std::array<double, 4>, 4> mat) {
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            std::cout << mat[i][j] << "   ";
        }
        std::cout << "\n";
    }
}