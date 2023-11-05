#ifndef HAWA_DATA_CONTAINERS_H
#define HAWA_DATA_CONTAINERS_H



#include <iostream>
#include <math.h>
#include <array>


struct StructPoseReal
{
    double x;  // meter
    double y;  // meter
    double yaw;  // radian

    StructPoseReal()
    {
    }

    StructPoseReal(double x_in, double y_in, double yaw_in)
    {
        x = x_in;
        y = y_in;
        yaw = yaw_in;
    }

    inline std::array<double, 3> to_array3()
    {
        return std::array<double, 3> {x, y, yaw};
    }

    inline void setFrom(StructPoseReal* in_ptr)
    {
        this->x = in_ptr->x;
        this->y = in_ptr->y;
        this->yaw = in_ptr->yaw;
    }
    
};


// inline void structPoseRealSetVal(StructPoseReal* target, const StructPoseReal* in_ptr)
// {
//     target->x = in_ptr->x;
//     target->y = in_ptr->y;
//     target->yaw = in_ptr->yaw;
// }

inline void structPoseRealReset(StructPoseReal* target_ptr)
{
    target_ptr->x = 0;
    target_ptr->y = 0;
    target_ptr->yaw = 0;
}




struct StructPoseGrid
{
    int x;
    int y;
    int yaw;

    StructPoseGrid()
    {
    }

    StructPoseGrid(int x, int y, int yaw):x(x), y(y), yaw(yaw)
    {
    }

    bool operator!=(const StructPoseGrid &p) const {
        return ! (x == p.x && y == p.y && yaw == p.yaw) ;
    }

    inline std::array<int, 3> to_array3()
    {
        return std::array<int, 3> {x, y, yaw};
    }

    inline std::array<int, 2> to_array2()
    {
        return std::array<int, 2> {x, y};
    }

};




#endif