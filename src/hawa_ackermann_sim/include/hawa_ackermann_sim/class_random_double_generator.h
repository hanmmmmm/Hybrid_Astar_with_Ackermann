
#ifndef HAWA_ACKERMANN_SIM_CLASS_RANDOM_DOUBLE_GENERATOR_H
#define HAWA_ACKERMANN_SIM_CLASS_RANDOM_DOUBLE_GENERATOR_H

#include <iostream>
#include <random>

namespace hawa 
{

/**
 * @brief Class to generate random double values in a given range
*/
class ClassRandomDoubleGenerator 
{
private:
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<double> dis;

    // Function to map a double value from range [minA, maxA] to range [minB, maxB]
    double map(double value, double minA, double maxA, double minB, double maxB) 
    {
        // Scale the value from range A to range [0, 1]
        double scaledValue = (value - minA) / (maxA - minA);
        
        // Map the scaled value to range B
        return minB + scaledValue * (maxB - minB);
    }
    
public:
    ClassRandomDoubleGenerator() : gen(rd()), dis(0.0, 1.0) {}
    
    // Function to generate a random double value in the range [min, max]
    inline double generate(double min, double max) 
    {
        double raw = dis(gen);
        return map(raw, 0.0, 1.0, min, max);
    }
};


}

#endif // HAWA_ACKERMANN_SIM_CLASS_RANDOM_DOUBLE_GENERATOR_H