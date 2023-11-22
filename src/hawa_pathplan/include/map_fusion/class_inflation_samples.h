// MIT License

// Copyright (c) 2023 Mingjie

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


/**
 * @file class_inflation_samples.h
 * @author Mingjie
 * @brief This is a class providing the gridmap inflation information. 
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_CLASS_GRIDMAP_INFLATION_SAMPLES
#define HAWA_CLASS_GRIDMAP_INFLATION_SAMPLES

#include <vector>
#include <iostream>

#include "ros/console.h"

/**
 * @brief This class contains the data for the inflation used in grid map. 
*/
class ClassInflationSamples
{
private:
    const int8_t cr = 0; // Occupancy value for the clear grid. 
    const int8_t v0 = 100; // Occupancy value for the highest occ score.
    const int8_t v1 = 95; // Occupancy value for the 2nd highest occ score.
    const int8_t v2 = 90; // Occupancy value for the 3rd highest occ score.
    const int8_t v3 = 85; // Occupancy value for the 4th highest occ score.
    const int8_t v4 = 80; // Occupancy value for the 5th highest occ score.
    const int8_t v5 = 75; // Occupancy value for the 6th highest occ score.

    int m_radius_val_;

    bool m_radius_is_set_;

public:
    ClassInflationSamples();
    ~ClassInflationSamples();

    // void prepareSampleByRadius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius);
    void prepareSampleByRadius();

    std::vector<std::vector<int8_t>> getInflationSample();

    std::vector< std::vector<int8_t> > m_inflate_sample_;

    void setRadius(const int rad);

    int getRadius();
};

ClassInflationSamples::ClassInflationSamples()
{
    m_radius_is_set_ = false;
}

ClassInflationSamples::~ClassInflationSamples()
{
}

/**
 * @brief Set the value of the radius. The value is in grid wise. 
 * @param rad The radius. It should be at least 1. Be careful about the maximum supported value. It can be found
 * in the function prepareSampleByRadius().
*/
void ClassInflationSamples::setRadius(const int rad)
{
    if (rad < 1)
    {
        ROS_ERROR_STREAM("setRadius() invalid radius:" << rad);
        return;
    }
    m_radius_val_ = rad;
    m_radius_is_set_ = true;
}

/**
 * @brief Get value of the radius. 
 * @return The radius.
*/
int ClassInflationSamples::getRadius()
{
    return m_radius_val_;
}

/**
 * @brief Call this function to get the inflation. 
 * @return The inflation sample. 
*/
std::vector<std::vector<int8_t>> ClassInflationSamples::getInflationSample()
{
    return m_inflate_sample_;
}

/**
 * @brief Generate the inflation sample, and save in a member variable.
*/
void ClassInflationSamples::prepareSampleByRadius()
{
    if (! m_radius_is_set_)
    {
        ROS_WARN_STREAM("Make sure radius is set first.");
        return;
    }

    m_inflate_sample_.clear();

    if( m_radius_val_ == 1)
    {
        m_inflate_sample_.push_back( {v2,v1,v2} );
        m_inflate_sample_.push_back( {v1,v0,v1} );
        m_inflate_sample_.push_back( {v2,v1,v2} );
    }
    else if( m_radius_val_ == 2)
    {
        m_inflate_sample_.push_back( {cr,v3,v2,v3,cr} );
        m_inflate_sample_.push_back( {v3,v1,v1,v1,v3} );
        m_inflate_sample_.push_back( {v2,v1,v0,v1,v2} );
        m_inflate_sample_.push_back( {v3,v1,v1,v1,v3} );
        m_inflate_sample_.push_back( {cr,v3,v2,v3,cr} );
    }
    else if( m_radius_val_ == 3)
    {
        m_inflate_sample_.push_back( {cr,cr,v3,v3,v3,cr,cr} );
        m_inflate_sample_.push_back( {cr,v2,v2,v2,v2,v2,cr} );
        m_inflate_sample_.push_back( {v3,v2,v1,v1,v1,v2,v3} );
        m_inflate_sample_.push_back( {v3,v2,v1,v0,v1,v2,v3} );
        m_inflate_sample_.push_back( {v3,v2,v1,v1,v1,v2,v3} );
        m_inflate_sample_.push_back( {cr,v2,v2,v2,v2,v2,cr} );
        m_inflate_sample_.push_back( {cr,cr,v3,v3,v3,cr,cr} );
    }
    else if( m_radius_val_ == 4)
    {
        m_inflate_sample_.push_back( {cr,cr,cr,v3,v3,v3,cr,cr,cr} );
        m_inflate_sample_.push_back( {cr,cr,v4,v3,v3,v3,v4,cr,cr} );
        m_inflate_sample_.push_back( {cr,v3,v2,v2,v2,v2,v2,v3,cr} );
        m_inflate_sample_.push_back( {v4,v3,v2,v1,v1,v1,v2,v3,v4} );
        m_inflate_sample_.push_back( {v4,v3,v2,v1,v0,v1,v2,v3,v4} );
        m_inflate_sample_.push_back( {v4,v3,v2,v1,v1,v1,v2,v3,v4} );
        m_inflate_sample_.push_back( {cr,v3,v2,v2,v2,v2,v2,v3,cr} );
        m_inflate_sample_.push_back( {cr,cr,v4,v3,v3,v3,v4,cr,cr} );
        m_inflate_sample_.push_back( {cr,cr,cr,v3,v3,v3,cr,cr,cr} );
    }
    else if( m_radius_val_ == 5)
    {
        m_inflate_sample_.push_back( {cr,cr,cr,cr,v5,v5,v5,cr,cr,cr,cr} );
        m_inflate_sample_.push_back( {cr,cr,cr,v5,v4,v4,v4,v5,cr,cr,cr} );
        m_inflate_sample_.push_back( {cr,cr,v5,v4,v3,v3,v3,v4,v5,cr,cr} );
        m_inflate_sample_.push_back( {cr,v5,v3,v2,v2,v2,v2,v2,v3,v5,cr} );
        m_inflate_sample_.push_back( {v5,v4,v3,v2,v1,v1,v1,v2,v3,v4,v5} );
        m_inflate_sample_.push_back( {v5,v4,v3,v2,v1,v0,v1,v2,v3,v4,v5} );
        m_inflate_sample_.push_back( {v5,v4,v3,v2,v1,v1,v1,v2,v3,v4,v5} );
        m_inflate_sample_.push_back( {cr,v5,v3,v2,v2,v2,v2,v2,v3,v5,cr} );
        m_inflate_sample_.push_back( {cr,cr,v5,v4,v3,v3,v3,v4,v5,cr,cr} );
        m_inflate_sample_.push_back( {cr,cr,cr,v5,v4,v4,v4,v5,cr,cr,cr} );
        m_inflate_sample_.push_back( {cr,cr,cr,cr,v5,v5,v5,cr,cr,cr,cr} );
    }
    else
    {
        ROS_ERROR_STREAM("Invalid inflation size:" << m_radius_val_);
    }
}







#endif