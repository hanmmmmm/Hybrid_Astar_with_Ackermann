
#include "class_tool_general.h"
#include <gtest/gtest.h>



TEST(calcOccValueTEST, test1)
{
    int8_t vmax = 100;
    int8_t vmin = 70;
    int radius = 2;
    int width = radius * 2 + 1;
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 0, radius, width), 57);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 1, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 2, radius, width), 70);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 3, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 4, radius, width), 57);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 0, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 1, radius, width), 78);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 2, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 3, radius, width), 78);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 4, radius, width), 66);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 0, radius, width), 70);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 1, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 2, radius, width), 100);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 3, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 4, radius, width), 70);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 0, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 1, radius, width), 78);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 2, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 3, radius, width), 78);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 4, radius, width), 66);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 0, radius, width), 57);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 1, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 2, radius, width), 70);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 3, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 4, radius, width), 57);
    
}


TEST(calcOccValueTEST, test2)
{
    int8_t vmax = 100;
    int8_t vmin = 70;
    int radius = 3;
    int width = radius * 2 + 1;
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 0, radius, width), 57);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 1, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 2, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 3, radius, width), 70);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 4, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 5, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 6, radius, width), 57);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 0, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 1, radius, width), 71);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 2, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 3, radius, width), 80);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 4, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 5, radius, width), 71);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 6, radius, width), 63);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 0, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 1, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 2, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 3, radius, width), 90);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 4, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 5, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 6, radius, width), 68);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 0, radius, width), 70);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 1, radius, width), 80);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 2, radius, width), 90);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 3, radius, width), 100);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 4, radius, width), 90);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 5, radius, width), 80);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 6, radius, width), 70);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 0, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 1, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 2, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 3, radius, width), 90);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 4, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 5, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 6, radius, width), 68);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 0, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 1, radius, width), 71);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 2, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 3, radius, width), 80);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 4, radius, width), 77);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 5, radius, width), 71);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 6, radius, width), 63);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 0, radius, width), 57);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 1, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 2, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 3, radius, width), 70);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 4, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 5, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 6, radius, width), 57);
    
}



TEST(calcOccValueTEST, test3)
{
    int8_t vmax = 100;
    int8_t vmin = 70;
    int radius = 6;
    int width = radius * 2 + 1;
    
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 0, radius, width), 57);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 1, 1, radius, width), 64);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 2, 2, radius, width), 71);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 3, 3, radius, width), 78);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 4, 4, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 5, 5, radius, width), 92);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 6, radius, width), 100);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 7, 7, radius, width), 92);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 8, 8, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 9, 9, radius, width), 78);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 10, 10, radius, width), 71);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 11, 11, radius, width), 64);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 12, 12, radius, width), 57);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 1, radius, width), 60);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 2, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 3, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 4, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 5, radius, width), 69);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 6, radius, width), 70);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 7, radius, width), 69);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 8, radius, width), 68);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 9, radius, width), 66);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 10, radius, width), 63);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 11, radius, width), 60);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 0, 12, radius, width), 57);

    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 1, radius, width), 75);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 2, radius, width), 80);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 3, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 4, radius, width), 90);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 5, radius, width), 95);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 6, radius, width), 100);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 7, radius, width), 95);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 8, radius, width), 90);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 9, radius, width), 85);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 10, radius, width), 80);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 11, radius, width), 75);
    EXPECT_EQ( hawa::classToolGeneral::calcOccValue(vmax, vmin, 6, 12, radius, width), 70);
    
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}