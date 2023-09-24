#ifndef CLASS_PATH_SEGMENT_CONTAINER_H
#define CLASS_PATH_SEGMENT_CONTAINER_H



#include <iostream>
#include <vector>
#include <math.h>

#include "../car_pathplan/struct_simple_pose.h"
// #include "../car_pathplan/class_custom_path.h"
// #include "../planner/conversion_tools.h"
// #include "helper_functions.h"


using std::cout;
using std::endl;
using std::vector;


class ClassPathSegmentContainer
{
private:
    vector<StructPoseReal> m_data_;


    
public:
    ClassPathSegmentContainer();
    ~ClassPathSegmentContainer();
};

ClassPathSegmentContainer::ClassPathSegmentContainer()
{
}

ClassPathSegmentContainer::~ClassPathSegmentContainer()
{
}











#endif