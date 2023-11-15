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
    int8_t v0;
    int8_t cr;
    int8_t v1;
    int8_t v2;
    int8_t v3;
    int8_t v4;
    int8_t v5;
public:
    ClassInflationSamples();
    ~ClassInflationSamples();

    void get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius);
};

ClassInflationSamples::ClassInflationSamples()
{
    v0 = 100;
    cr = 0; 
    v1 = 95;
    v2 = 90;
    v3 = 85;
    v4 = 80;
    v5 = 75;
}

ClassInflationSamples::~ClassInflationSamples()
{
}

void ClassInflationSamples::get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, 
                                                 const int radius){
    if( radius == 1){
        inflate_sample_target.push_back( {v2, v1, v2} );
        inflate_sample_target.push_back( {v1, v0, v1} );
        inflate_sample_target.push_back( {v2, v1, v2} );
    }
    else if( radius == 2){
        inflate_sample_target.push_back( {cr, v3, v2, v3, cr} );
        inflate_sample_target.push_back( {v3, v1, v1, v1, v3} );
        inflate_sample_target.push_back( {v2, v1, v0, v1, v2} );
        inflate_sample_target.push_back( {v3, v1, v1, v1, v3} );
        inflate_sample_target.push_back( {cr, v3, v2, v3, cr} );
    }
    else if( radius == 3){
        inflate_sample_target.push_back( {cr,cr,v3,v3,v3,cr,cr} );
        inflate_sample_target.push_back( {cr,v2,v2,v2,v2,v2,cr} );
        inflate_sample_target.push_back( {v3,v2,v1,v1,v1,v2,v3} );
        inflate_sample_target.push_back( {v3,v2,v1,v0,v1,v2,v3} );
        inflate_sample_target.push_back( {v3,v2,v1,v1,v1,v2,v3} );
        inflate_sample_target.push_back( {cr,v2,v2,v2,v2,v2,cr} );
        inflate_sample_target.push_back( {cr,cr,v3,v3,v3,cr,cr} );
    }
    else if( radius == 4){
        inflate_sample_target.push_back( {cr,cr,cr,v3,v3,v3,cr,cr,cr} );
        inflate_sample_target.push_back( {cr,cr,v4,v3,v3,v3,v4,cr,cr} );
        inflate_sample_target.push_back( {cr,v3,v2,v2,v2,v2,v2,v3,cr} );
        inflate_sample_target.push_back( {v4,v3,v2,v1,v1,v1,v2,v3,v4} );
        inflate_sample_target.push_back( {v4,v3,v2,v1,v0,v1,v2,v3,v4} );
        inflate_sample_target.push_back( {v4,v3,v2,v1,v1,v1,v2,v3,v4} );
        inflate_sample_target.push_back( {cr,v3,v2,v2,v2,v2,v2,v3,cr} );
        inflate_sample_target.push_back( {cr,cr,v4,v3,v3,v3,v4,cr,cr} );
        inflate_sample_target.push_back( {cr,cr,cr,v3,v3,v3,cr,cr,cr} );
    }
    else if( radius >= 5){
        inflate_sample_target.push_back( {cr,cr,cr,cr,v5,v5,v5,cr,cr,cr,cr} );
        inflate_sample_target.push_back( {cr,cr,cr,v4,v4,v4,v4,v4,cr,cr,cr} );
        inflate_sample_target.push_back( {cr,cr,v4,v4,v3,v3,v3,v4,v4,cr,cr} );
        inflate_sample_target.push_back( {cr,v4,v3,v2,v2,v2,v2,v2,v3,v4,cr} );
        inflate_sample_target.push_back( {v5,v4,v3,v2,v1,v1,v1,v2,v3,v4,v5} );
        inflate_sample_target.push_back( {v5,v4,v3,v2,v1,v0,v1,v2,v3,v4,v5} );
        inflate_sample_target.push_back( {v5,v4,v3,v2,v1,v1,v1,v2,v3,v4,v5} );
        inflate_sample_target.push_back( {cr,v4,v3,v2,v2,v2,v2,v2,v3,v4,cr} );
        inflate_sample_target.push_back( {cr,cr,v5,v4,v3,v3,v3,v4,v5,cr,cr} );
        inflate_sample_target.push_back( {cr,cr,cr,v4,v4,v4,v4,v4,cr,cr,cr} );
        inflate_sample_target.push_back( {cr,cr,cr,cr,v5,v5,v5,cr,cr,cr,cr} );
    }
}







#endif