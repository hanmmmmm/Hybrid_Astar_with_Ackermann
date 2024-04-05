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
 * @file bresenham.h
 * @author Mingjie
 * @brief This is an implementation of the Bresenham algorithm used for find the grids 
 * connecting between two grids. 
 * @version 0.1
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef BRESENHAM_H
#define BRESENHAM_H

#include "common_includes.h"

namespace hawa
{

/**
 * @brief This class is an implementation of the Bresenham algorithm used for find the grids.
*/
class ClassUtilsBresenham
{
private:

    static int bresenhamSign(int dxy)
    {
        if(dxy<0) return -1; 
        else if(dxy>0) return 1; 
        else return 0;
    }

public:
    ClassUtilsBresenham() {};
    ~ClassUtilsBresenham() {};

    static std::vector< std::array<int,2> > solve(int x1, int y1, int x2, int y2)
    {
        std::vector<std::array<int,2>> out;

        int Dx = x2 - x1;
        int Dy = y2 - y1;

        //# Increments
        int Sx = bresenhamSign(Dx); 
        int Sy = bresenhamSign(Dy);

        //# Segment length
        Dx = abs(Dx); 
        Dy = abs(Dy); 
        int D = std::max(Dx, Dy);

        //# Initial remainder
        double R = D / 2;

        int X = x1;
        int Y = y1;
        if(Dx > Dy)
        {   
            for(int I=0; I<D; I++)
            {   
                std::array<int,2> one_px;
                // one_px.push_back(X);
                // one_px.push_back(Y);
                one_px[0] = X;
                one_px[1] = Y;
                //# Update (X, Y) and R
                X += Sx; 
                R += Dy; //# Lateral move
                if (R >= Dx)
                {
                    Y += Sy; 
                    R -= Dx; //# Diagonal move
                }
                out.push_back( one_px );
            }
            
        }
        else
        {   
            for(int I=0; I<D; I++)
            {    
                std::array<int,2> one_px;
                // one_px.push_back(X);
                // one_px.push_back(Y);
                one_px[0] = X;
                one_px[1] = Y;
                //# Update (X, Y) and R
                Y += Sy; 
                R += Dx; //# Lateral move
                if(R >= Dy)
                {    
                    X += Sx; 
                    R -= Dy; //# Diagonal move
                }
                out.push_back( one_px );
            }
        }
        return out;
    }

};





} // namespace hawa

#endif