#include <iostream>
#include <iomanip>
#include <array>
#include <cmath>

#include "matrixutils.hpp"

#define PI 3.14159265359

std::array<std::array<double, 4>, 4> forwardkinematics(std::array<double, 6> angle, std::array<double, 7> link) {

    std::array<std::array<double, 4>, 4> H0 = {{
        {cos(angle[0]), -sin(angle[0]), 0, 0},
        {sin(angle[0]), cos(angle[0]), 0, 0},
        {0, 0, 1, link[0]},
        {0, 0, 0, 1}
    }};
    std::array<std::array<double, 4>, 4> H1 = {{
        {cos(angle[1]), 0, sin(angle[1]), 0},
        {0, 1, 0, 0},
        {-sin(angle[1]), 0, cos(angle[1]), link[1]},
        {0, 0, 0, 1}
    }};
    std::array<std::array<double, 4>, 4> H2 = {{
        {cos(angle[2]), 0, sin(angle[2]), 0},
        {0, 1, 0, 0},
        {-sin(angle[2]), 0, cos(angle[2]), link[2]},
        {0, 0, 0, 1}
    }};
    std::array<std::array<double, 4>, 4> H3 = {{
        {1, 0, 0, link[4]},
        {0, cos(angle[3]), -sin(angle[3]), 0},
        {0, sin(angle[3]), cos(angle[3]), link[3]},
        {0, 0, 0, 1}
    }};
    std::array<std::array<double, 4>, 4> H4 = {{
        {cos(angle[4]), 0, sin(angle[4]), link[5]},
        {0, 1, 0, 0},
        {-sin(angle[4]), 0, cos(angle[4]), 0},
        {0, 0, 0, 1}
    }};
    std::array<std::array<double, 4>, 4> H5 = {{
        {1, 0, 0, link[6]},
        {0, cos(angle[5]), -sin(angle[5]), 0},
        {0, sin(angle[5]), cos(angle[5]), 0},
        {0, 0, 0, 1}
    }};
    std::array<std::array<double, 4>, 4> H6 = {{
        {cos(PI / 2), 0, sin(PI / 2), 0},
        {0, 1, 0, 0},
        {-sin(PI / 2), 0, cos(PI / 2), 0},
        {0, 0, 0, 1}
    }};

    std::array<std::array<double, 4>, 4> H56 = MatMultiply4(H5, H6);
    std::array<std::array<double, 4>, 4> H46 = MatMultiply4(H4, H56);
    std::array<std::array<double, 4>, 4> H36 = MatMultiply4(H3, H46);
    std::array<std::array<double, 4>, 4> H26 = MatMultiply4(H2, H36);
    std::array<std::array<double, 4>, 4> H16 = MatMultiply4(H1, H26);
    std::array<std::array<double, 4>, 4> H06 = MatMultiply4(H0, H16);

    return H06;
}