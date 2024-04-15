
#include "common/tools_angles.h"
#include <gtest/gtest.h>

using namespace hawa;

TEST(mod2piTEST, test1)
{
    EXPECT_EQ( hawa::Tools::mod2pi(0), 0);
    EXPECT_EQ( hawa::Tools::mod2pi(1.2), 1.2);
    EXPECT_EQ( hawa::Tools::mod2pi(M_PI), M_PI);
    EXPECT_EQ( hawa::Tools::mod2pi(M_PI * 2), M_PI*2);
    EXPECT_EQ( hawa::Tools::mod2pi(M_PI * 3), M_PI);
    EXPECT_EQ( hawa::Tools::mod2pi(-M_PI), M_PI);
    EXPECT_EQ( hawa::Tools::mod2pi(-M_PI * 2), 0);
    EXPECT_EQ( hawa::Tools::mod2pi(-M_PI * 3), M_PI);
    EXPECT_EQ( hawa::Tools::mod2pi(-M_PI * 4), 0);
}

TEST(computeYawOf2PointsTEST, test2)
{
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, 1, 0), 0);
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, 0, 1), M_PI/2);
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, -1, 0), M_PI);
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, 0, -1), -M_PI/2);
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, 1, 1), M_PI/4);
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, -1, 1), M_PI*3/4);
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, -1, -1), -M_PI*3/4);
    EXPECT_EQ( hawa::Tools::computeYawOf2Points(0, 0, 1, -1), -M_PI/4);
}

TEST(checkIfPointOnLeftOfLineTEST, test3)
{
    EXPECT_EQ( hawa::Tools::checkIfPointOnLeftOfLine(0, 0, 1, 0, 0, 1), true);
    EXPECT_EQ( hawa::Tools::checkIfPointOnLeftOfLine(0, 0, 1, 0, 0, -1), false);
    EXPECT_EQ( hawa::Tools::checkIfPointOnLeftOfLine(0, 0, 1, 0, 1, 1), true);
    EXPECT_EQ( hawa::Tools::checkIfPointOnLeftOfLine(0, 0, 1, 0, 1, -1), false);
    EXPECT_EQ( hawa::Tools::checkIfPointOnLeftOfLine(0, 0, 1, 0, -1, 1), false);
    EXPECT_EQ( hawa::Tools::checkIfPointOnLeftOfLine(0, 0, 1, 0, -1, -1), true);
}

TEST(calcAngleByThreePointsTEST, test4)
{
    std::array<double,3> p1 = {0, 0, 0};
    std::array<double,3> p2 = {1, 0, 0};
    std::array<double,3> p3 = {1, 1, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI/2);
    p1 = {0, 0, 0};
    p2 = {1, 0, 0};
    p3 = {1, -1, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI/2);
    p1 = {0, 0, 0};
    p2 = {1, 0, 0};
    p3 = {0, 1, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI/4);
    p1 = {0, 0, 0};
    p2 = {1, 0, 0};
    p3 = {0, -1, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI/4);
    p1 = {0, 0, 0};
    p2 = {1, 0, 0};
    p3 = {2, 1, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI * 0.75);
    p1 = {0, 0, 0};
    p2 = {1, 0, 0};
    p3 = {2, -1, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI * 0.75);
    p1 = {0, 0, 0};
    p2 = {1, 0, 0};
    p3 = {2, 0, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI);
    p1 = {0, 0, 0};
    p2 = {1, 1, 0};
    p3 = {2, 2, 0};
    EXPECT_EQ( hawa::Tools::calcAngleByThreePoints(p1, p2, p3), M_PI);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}