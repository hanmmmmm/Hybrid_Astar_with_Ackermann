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
#include "class_tool_general.h"

namespace hawa
{
    
/**
 * @brief This class contains the data for the inflation used in grid map. 
*/
class ClassInflationSamples
{
private:
    const int8_t vclr = 0; // Occupancy value for the clear grid. 
    const int8_t vmax = 100; // Occupancy value for the highest occ score.


    int8_t m_min_occ_val_;

    int m_radius_, m_width_;

    std::vector< std::vector<int8_t> > m_inflate_sample_;
    
    void fillSample();

    void printSample();


public:
    ClassInflationSamples(const int min_occ_val = 70, const int radius = 4);
    ~ClassInflationSamples();

    std::vector< std::vector<int8_t> > getInflationSample();

};
}






#endif