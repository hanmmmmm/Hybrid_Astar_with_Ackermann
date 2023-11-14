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


#ifndef HAWA_CLASS_GRIDMAP_INFLATION_SAMPLES
#define HAWA_CLASS_GRIDMAP_INFLATION_SAMPLES

#include <vector>
#include <iostream>

class ClassInflationSamples
{
private:
    int8_t obs_0;
    int8_t clr__;
    int8_t obs_1;
    int8_t obs_2;
    int8_t obs_3;
public:
    ClassInflationSamples();
    ~ClassInflationSamples();

    void get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius);
};

ClassInflationSamples::ClassInflationSamples()
{
    obs_0 = 100;
    clr__ = 0; 
    obs_1 = 90;
    obs_2 = 80;
    obs_3 = 70;
}

ClassInflationSamples::~ClassInflationSamples()
{
}

void ClassInflationSamples::get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, 
                                                 const int radius){
    if( radius == 1){
        inflate_sample_target.push_back( {obs_2, obs_1, obs_2} );
        inflate_sample_target.push_back( {obs_1, obs_0, obs_1} );
        inflate_sample_target.push_back( {obs_2, obs_1, obs_2} );
    }
    else if( radius == 2){
        inflate_sample_target.push_back( {clr__, obs_3, obs_2, obs_3, clr__} );
        inflate_sample_target.push_back( {obs_3, obs_1, obs_1, obs_1, obs_3} );
        inflate_sample_target.push_back( {obs_2, obs_1, obs_0, obs_1, obs_2} );
        inflate_sample_target.push_back( {obs_3, obs_1, obs_1, obs_1, obs_3} );
        inflate_sample_target.push_back( {clr__, obs_3, obs_2, obs_3, clr__} );
    }
    else if( radius == 3){
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_0,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
    }
    else if( radius >= 4){
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_0,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
    }
}







#endif