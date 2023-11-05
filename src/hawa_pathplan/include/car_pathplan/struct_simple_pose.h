#ifndef STRUCT_SIMPLE_POSE_H
#define STRUCT_SIMPLE_POSE_H

#include <iostream>
#include <math.h>



// struct StructPoseReal
// {
//     double x;  // meter
//     double y;  // meter
//     double yaw;  // radian

//     StructPoseReal()
//     {
//     }

//     StructPoseReal(double x_in, double y_in, double yaw_in)
//     {
//         x = x_in;
//         y = y_in;
//         yaw = yaw_in;
//     }

//     inline std::array<double, 3> to_array3()
//     {
//         return std::array<double, 3> {x, y, yaw};
//     }
    
// };

// struct StructPoseGrid
// {
//     int x;
//     int y;
//     int yaw;

//     // `operator==` is required to compare keys in case of a hash collision
//     bool operator==(const StructPoseGrid &p) const {
//         return x == p.x && y == p.y && yaw == p.yaw;
//     }

//     bool operator!=(const StructPoseGrid &p) const {
//         return ! (x == p.x && y == p.y && yaw == p.yaw) ;
//     }

// };



// struct StructPoseGrid
// {
//     int x;
//     int y;
//     int yaw;

//     StructPoseGrid()
//     {
//     }

//     StructPoseGrid(int x, int y, int yaw):x(x), y(y), yaw(yaw)
//     {
//     }

//     bool operator!=(const StructPoseGrid &p) const {
//         return ! (x == p.x && y == p.y && yaw == p.yaw) ;
//     }

//     inline std::array<int, 3> to_array3()
//     {
//         return std::array<int, 3> {x, y, yaw};
//     }

//     inline std::array<int, 2> to_array2()
//     {
//         return std::array<int, 2> {x, y};
//     }

// };


struct StructPoseGrid2D
{
    int x;
    int y;

    StructPoseGrid2D()
    {
    }

    StructPoseGrid2D(int x, int y, int yaw):x(x), y(y)
    {
    }

    bool operator!=(const StructPoseGrid2D &p) const {
        return ! (x == p.x && y == p.y) ;
    }

    inline std::array<int, 2> to_array2()
    {
        return std::array<int, 2> {x, y};
    }

};

template <typename T>
struct StructValueInXY
{
    T x;
    T y;
};


struct StructWaypointWithTwist
{
    double x, y, yaw;

    StructValueInXY<double> body_v;
    StructValueInXY<double> body_a;
    StructValueInXY<double> body_j;
    
    StructValueInXY<double> fix_v;
    StructValueInXY<double> fix_a;
    StructValueInXY<double> fix_j;

    // double body_vx, body_vy;  // x is along robot center longitude, y is lateral 
    // double body_ax, body_jx;  // a is acceleration of v, j is rate of change of a.
    // double body_ay, body_jy;  // a is acceleration of v, j is rate of change of a.

    // double fix_vx, fix_vy;  // values in fix frame, e.g ground or /map
    // double fix_ax, fix_ay;
    // double fix_jx, fix_jy;

    double angular, d_angular;  // w is angular speed, wr is rate of change of w.

    StructWaypointWithTwist()
    {
        x = 0;
        y = 0;
        yaw = 0;
        
        // body_vx = 0;
        // body_vy = 0;
        // body_ax = 0;
        // body_ay = 0;
        // body_jx = 0;
        // body_jy = 0;

        // fix_vx = 0;
        // fix_vy = 0;
        // fix_ax = 0;
        // fix_ay = 0;
        // fix_jx = 0;
        // fix_jy = 0;

        angular = 0;
        d_angular = 0;
    }

    // void update_fix_from_body()
    // {
    //     fix_vx = body_vx * cos(yaw);
    //     fix_vy = body_vx * sin(yaw);
    //     fix_ax = body_ax * cos(yaw);
    //     fix_ay = body_ax * sin(yaw);
    //     fix_jx = body_jx * cos(yaw);
    //     fix_jy = body_jx * sin(yaw);
    // }

    std::array<double, 3> pose_array()
    {
        return std::array<double, 3>{x,y,yaw};
    }

};


// StructWaypointWithTwist project_future_waypoint_new_point(StructWaypointWithTwist & wp, double t_sec)
// {
//     wp.update_fix_from_body();
//     double _ax_change = wp.fix_jx * t_sec / 2.0;
//     double _ay_change = wp.fix_jy * t_sec / 2.0;

//     double _new_ax = wp.fix_ax + _ax_change;
//     double _new_ay = wp.fix_ay + _ay_change;

//     double _new_vx = wp.fix_vx + _new_ax * t_sec;
//     double _new_vy = wp.fix_vy + _new_ay * t_sec;

//     double _new_x = wp.x + _new_vx * t_sec;
//     double _new_y = wp.y + _new_vy * t_sec;

//     double _angular_change = wp.d_angular * t_sec;
//     double _new_angular = wp.angular + _angular_change;
//     double _new_yaw = wp.yaw + _new_angular * t_sec;

//     StructWaypointWithTwist _result;
//     _result.x = _new_x;
//     _result.y = _new_y;
//     _result.yaw = _new_yaw;

//     _result.fix_vx = 


// }


// h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);


// struct hash_function_for_StructPoseGrid
// {
//     std::size_t operator() (const StructPoseGrid &node) const
//     {
//         std::size_t h = 0;
//         h ^= std::hash<int>{}(node.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
//         h ^= std::hash<int>{}(node.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
//         h ^= std::hash<int>{}(node.yaw) + 0x9e3779b9 + (h << 6) + (h >> 2);
//         return h;
//     }

//     // template <class T1, class T2>
//     // std::size_t operator() (const Node<T1, T2> &node) const
//     // {
//     //     std::size_t h1 = std::hash<T1>()(node.x);
//     //     std::size_t h2 = std::hash<T2>()(node.y);
 
//     //     return h1 ^ h2;
//     // }
// };


struct StructRectangle
{
    double x_up=0;
    double y_up=0;
    double x_dw=0;
    double y_dw=0;
};

#endif