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
 * This file is 
 */


#ifndef CLASS_HAWA_REEDSHEPP_PATHS__
#define CLASS_HAWA_REEDSHEPP_PATHS__

#include <string>
#include <vector>
#include <array>

#include "../reedsshepp_tools.h"

class ClassReedSheppPath
{
private:

    
public:
    ClassReedSheppPath();
    ~ClassReedSheppPath();

    struct posePerSample
    {
        posePerSample(double xp, double yp, double angle):x(xp),y(yp),theta(angle){};
        double x;
        double y;
        double theta; // radian
    };
    

    
    
    struct PathCollection
    {
        StructRSPathResult LpSpLp; 
        StructRSPathResult LpSpRp;
        StructRSPathResult LmSmLm;
        StructRSPathResult LmSmRm;
        StructRSPathResult RpSpRp;
        StructRSPathResult RpSpLp;
        StructRSPathResult RmSmRm;
        StructRSPathResult RmSmLm;

        StructRSPathResult RpLmRp;
        StructRSPathResult RmLpRm;
        StructRSPathResult RpLmRm;
        StructRSPathResult RmLpRp;
        StructRSPathResult RpLpRp;
        StructRSPathResult RpLpRm;
        StructRSPathResult RmLmRp;
        StructRSPathResult RmLmRm;

        StructRSPathResult LpRpLp;
        StructRSPathResult LpRpLm;
        StructRSPathResult LpRmLp;
        StructRSPathResult LpRmLm;
        StructRSPathResult LmRpLp;
        StructRSPathResult LmRpLm;
        StructRSPathResult LmRmLp;
        StructRSPathResult LmRmLm;

        StructRSPathResult LpRupLumRm;
        StructRSPathResult LmRumLupRp;
        StructRSPathResult RpLupRumLm;
        StructRSPathResult RmLumRupLp;
        StructRSPathResult LpRumLumRp;
        StructRSPathResult LmRupLupRm;
        StructRSPathResult RpLumRumLp;
        StructRSPathResult RmLupRupLm;

        StructRSPathResult LpRm90SmRm;
        StructRSPathResult LmRp90SpRp;
        StructRSPathResult RpLm90SmLm;
        StructRSPathResult RmLp90SpLp;

        StructRSPathResult LpRm90SmLm;
        StructRSPathResult LmRp90SpLp;
        StructRSPathResult RpLm90SmRm;
        StructRSPathResult RmLp90SpRp;

        StructRSPathResult RpSpLp90Rm;
        StructRSPathResult RmSmLm90Rp;
        StructRSPathResult LpSpRp90Lm;
        StructRSPathResult LmSmRm90Lp;

        StructRSPathResult LpSpLp90Rm;
        StructRSPathResult LmSmLm90Rp;
        StructRSPathResult RpSpRp90Lm;
        StructRSPathResult RmSmRm90Lp;

        StructRSPathResult LpRm90SmLm90Rp;
        StructRSPathResult LmRp90SpLp90Rm;
        StructRSPathResult RpLm90SmRm90Lp;
        StructRSPathResult RmLp90SpRp90Lm;

        PathCollection()
        {
            LpSpLp.setName("LpSpLp");
            LpSpLp.setName("LpSpLp"); 
            LpSpRp.setName("LpSpRp");
            LmSmLm.setName("LmSmLm");
            LmSmRm.setName("LmSmRm");
            RpSpRp.setName("RpSpRp");
            RpSpLp.setName("RpSpLp");
            RmSmRm.setName("RmSmRm");
            RmSmLm.setName("RmSmLm");

            RpLmRp.setName("RpLmRp");
            RmLpRm.setName("RmLpRm");
            RpLmRm.setName("RpLmRm");
            RmLpRp.setName("RmLpRp");
            RpLpRp.setName("RpLpRp");
            RpLpRm.setName("RpLpRm");
            RmLmRp.setName("RmLmRp");
            RmLmRm.setName("RmLmRm");

            LpRpLp.setName("LpRpLp");
            LpRpLm.setName("LpRpLm");
            LpRmLp.setName("LpRmLp");
            LpRmLm.setName("LpRmLm");
            LmRpLp.setName("LmRpLp");
            LmRpLm.setName("LmRpLm");
            LmRmLp.setName("LmRmLp");
            LmRmLm.setName("LmRmLm");

            LpRupLumRm.setName("LpRupLumRm");
            LmRumLupRp.setName("LmRumLupRp");
            RpLupRumLm.setName("RpLupRumLm");
            RmLumRupLp.setName("RmLumRupLp");
            LpRumLumRp.setName("LpRumLumRp");
            LmRupLupRm.setName("LmRupLupRm");
            RpLumRumLp.setName("RpLumRumLp");
            RmLupRupLm.setName("RmLupRupLm");

            LpRm90SmRm.setName("LpRm90SmRm");
            LmRp90SpRp.setName("LmRp90SpRp");
            RpLm90SmLm.setName("RpLm90SmLm");
            RmLp90SpLp.setName("RmLp90SpLp");

            LpRm90SmLm.setName("LpRm90SmLm");
            LmRp90SpLp.setName("LmRp90SpLp");
            RpLm90SmRm.setName("RpLm90SmRm");
            RmLp90SpRp.setName("RmLp90SpRp");

            RpSpLp90Rm.setName("RpSpLp90Rm");
            RmSmLm90Rp.setName("RmSmLm90Rp");
            LpSpRp90Lm.setName("LpSpRp90Lm");
            LmSmRm90Lp.setName("LmSmRm90Lp");

            LpSpLp90Rm.setName("LpSpLp90Rm");
            LmSmLm90Rp.setName("LmSmLm90Rp");
            RpSpRp90Lm.setName("RpSpRp90Lm");
            RmSmRm90Lp.setName("RmSmRm90Lp");

            LpRm90SmLm90Rp.setName("LpRm90SmLm90Rp");
            LmRp90SpLp90Rm.setName("LmRp90SpLp90Rm");
            RpLm90SmRm90Lp.setName("RpLm90SmRm90Lp");
            RmLp90SpRp90Lm.setName("RmLp90SpRp90Lm");
        }

        

        void resetAllPaths(){
            LpSpLp.reset();
            LpSpRp.reset();
            LmSmLm.reset();
            LmSmRm.reset();
            RpSpRp.reset();
            RpSpLp.reset();
            RmSmRm.reset();
            RmSmLm.reset();

            LpRpLp.reset();
            LpRpLm.reset();
            LpRmLp.reset();
            LmRpLm.reset();
            LpRmLm.reset();            
            LmRpLp.reset();            
            LmRmLp.reset();            
            LmRmLm.reset();        

            RpLpRp.reset();
            RpLmRp.reset();
            RmLpRm.reset();
            RpLmRm.reset();
            RmLpRp.reset();
            RpLpRm.reset();
            RmLmRp.reset();
            RmLmRm.reset();
            
            LpRupLumRm.reset();
            LmRumLupRp.reset();
            RpLupRumLm.reset();
            RmLumRupLp.reset();
            LpRumLumRp.reset();
            LmRupLupRm.reset();
            RpLumRumLp.reset();
            RmLupRupLm.reset();

            LpRm90SmRm.reset();
            LmRp90SpRp.reset();
            RpLm90SmLm.reset();
            RmLp90SpLp.reset();

            LpRm90SmLm.reset();
            LmRp90SpLp.reset();
            RpLm90SmRm.reset();
            RmLp90SpRp.reset();


            RpSpLp90Rm.reset();
            RmSmLm90Rp.reset();
            LpSpRp90Lm.reset();
            LmSmRm90Lp.reset();

            LpSpLp90Rm.reset();
            LmSmLm90Rp.reset();
            RpSpRp90Lm.reset();
            RmSmRm90Lp.reset();


            LpRm90SmLm90Rp.reset();
            LmRp90SpLp90Rm.reset();
            RpLm90SmRm90Lp.reset();
            RmLp90SpRp90Lm.reset();
        }
    };
};

ClassReedSheppPath::ClassReedSheppPath()
{
}

ClassReedSheppPath::~ClassReedSheppPath()
{
}



#endif