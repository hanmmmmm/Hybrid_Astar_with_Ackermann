
#ifndef HAWA_ACKERMANN_SIM_CLASS_RANDOM_SIN_NOISE_GENERATOR_H
#define HAWA_ACKERMANN_SIM_CLASS_RANDOM_SIN_NOISE_GENERATOR_H

#include <iostream> 
#include "class_random_double_generator.h"
#include "hawa_sim_tools.h"


namespace hawa
{

struct SinNoiseParameters
{
    double amplitude;
    double frequency;
    double phase;
    double offset;
    int type = 0; // 0: x, 1: y, 2: yaw
};


class ClassRandomSinNoiseGenerator
{
private:
    SinNoiseParameters m_parameters_x_, m_parameters_y_, m_parameters_yaw_;

    double m_time_;

    double m_last_restart_time_;

    bool m_initialized_ = false;

    ClassRandomDoubleGenerator m_random_double_generator_; 

    void updateTime();
    bool needNewStart();
    void generateNewParameters(SinNoiseParameters &parameters);
    double calculateNoise(const SinNoiseParameters &parameters);

public:
    ClassRandomSinNoiseGenerator();
    void getNoise(double &x, double &y, double &yaw);


};

}


#endif // HAWA_ACKERMANN_SIM_CLASS_RANDOM_SIN_NOISE_GENERATOR_H