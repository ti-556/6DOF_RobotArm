#include <iostream>
#include <array>
#include <cmath>

#include "includes/IKwithlength.hpp"

int main() {

//homogeneous transformation matrix of endeffector which respresents both position and orientation
std::array<std::array<double, 4>, 4> endeffector = {{
    {-0.9696, 0.1344, 0.2043, 293.2939},
    {0.2096, 0.8870, 0.4114, 224.6901},
    {-0.1259, 0.4418, -0.8883, 293.4121},
    {0, 0, 0, 1}
}};

// link of link lengths
std::array<double, 7> linklist = {187, 103, 270, 70, 134, 168, 72};

std::array<double, 4> wristeffector = wristcalculation(endeffector, linklist);

std::array<double, 2> angle1 = angle1calculation(wristeffector);
std::array<double, 4> angle2 = angle2calculation(wristeffector, angle1, linklist);
std::array<double, 4> angle3 = angle3calculation(wristeffector, angle2, angle1, linklist);


std::array<std::array<double, 3>, 3> R3toE_AA = Rotation3toE(endeffector, angle1[0], angle2[0], angle3[0]);
std::array<double, 2> angle5AA = angle5calculation(R3toE_AA);
std::array<double, 2> angle4AA = angle4calculation(R3toE_AA, angle5AA);
std::array<double, 2> angle6AA = angle6calculation(R3toE_AA, angle5AA);


std::array<std::array<double, 3>, 3> R3toE_AB = Rotation3toE(endeffector, angle1[0], angle2[1], angle3[1]);
std::array<double, 2> angle5AB = angle5calculation(R3toE_AB);
std::array<double, 2> angle4AB = angle4calculation(R3toE_AB, angle5AB);
std::array<double, 2> angle6AB = angle6calculation(R3toE_AB, angle5AB);


std::array<std::array<double, 3>, 3> R3toE_BA = Rotation3toE(endeffector, angle1[1], angle2[2], angle3[2]);
std::array<double, 2> angle5BA = angle5calculation(R3toE_BA);
std::array<double, 2> angle4BA = angle4calculation(R3toE_BA, angle5BA);
std::array<double, 2> angle6BA = angle6calculation(R3toE_BA, angle5BA);


std::array<std::array<double, 3>, 3> R3toE_BB = Rotation3toE(endeffector, angle1[1], angle2[3], angle3[3]);
std::array<double, 2> angle5BB = angle5calculation(R3toE_BB);
std::array<double, 2> angle4BB = angle4calculation(R3toE_BB, angle5BB);
std::array<double, 2> angle6BB = angle6calculation(R3toE_BB, angle5BB);

std::cout << "----- angle results -----" << std::endl;

std::cout << std::fixed << "angle 1A: " << angle1[0] * 180 / PI << " angle 1B: " << angle1[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 2AA: " << angle2[0] * 180 / PI << " angle 2AB: " << angle2[1] * 180 / PI << " angle 2BA: " << angle2[2] * 180 / PI << " angle 2BB: " << angle2[3] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 3AA: " << angle3[0] * 180 / PI << " angle 3AB: " << angle3[1] * 180 / PI << " angle 3BA: " << angle3[2] * 180 / PI << " angle 3BB: " << angle3[3] * 180 / PI << " [degrees]" << std::endl;

std::cout << "------------------------------------------" << std::endl;

std::cout << std::fixed << "angle 4 AAA: " << angle4AA[0] * 180 / PI << " angle4 AAB: " << angle4AA[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 5 AAA: " << angle5AA[0] * 180 / PI << " angle5 AAB: " << angle5AA[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 6 AAA: " << angle6AA[0] * 180 / PI << " angle6 AAB: " << angle6AA[1] * 180 / PI << " [degrees]" << std::endl;

std::cout << "------------------------------------------" << std::endl;

std::cout << std::fixed << "angle 4 ABA: " << angle4AB[0] * 180 / PI << " angle4 ABB: " << angle4AB[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 5 ABA: " << angle5AB[0] * 180 / PI << " angle5 ABB: " << angle5AB[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 6 ABA: " << angle6AB[0] * 180 / PI << " angle6 ABB: " << angle6AB[1] * 180 / PI << " [degrees]" << std::endl;

std::cout << "------------------------------------------" << std::endl;

std::cout << std::fixed << "angle 4 BAA: " << angle4BA[0] * 180 / PI << " angle4 BAB: " << angle4BA[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 5 BAA: " << angle5BA[0] * 180 / PI << " angle5 BAB: " << angle5BA[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 6 BAA: " << angle6BA[0] * 180 / PI << " angle6 BAB: " << angle6BA[1] * 180 / PI << " [degrees]" << std::endl;

std::cout << "------------------------------------------" << std::endl;

std::cout << std::fixed << "angle 4 BBA: " << angle4BB[0] * 180 / PI << " angle4 BBB: " << angle4BB[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 5 BBA: " << angle5BB[0] * 180 / PI << " angle5 BBB: " << angle5BB[1] * 180 / PI << " [degrees]" << std::endl;
std::cout << std::fixed << "angle 6 BBA: " << angle6BB[0] * 180 / PI << " angle6 BBB: " << angle6BB[1] * 180 / PI << " [degrees]" << std::endl;

return 0;
}