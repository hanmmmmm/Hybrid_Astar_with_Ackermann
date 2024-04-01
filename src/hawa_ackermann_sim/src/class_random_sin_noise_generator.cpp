
#include "hawa_ackermann_sim/class_random_sin_noise_generator.h"


namespace hawa
{

/**
 * @brief Default Constructor
*/
ClassRandomSinNoiseGenerator::ClassRandomSinNoiseGenerator() 
{
    m_time_ = 0;
    m_last_restart_time_ = 0;
}

/**
 * @brief Get the sin noise values
*/
void ClassRandomSinNoiseGenerator::getNoise(double &x, double &y, double &yaw)
{
    if (needNewStart())
    {
        generateNewParameters(m_parameters_x_);
        generateNewParameters(m_parameters_y_);
        generateNewParameters(m_parameters_yaw_);
    }
    updateTime();
    x = calculateNoise(m_parameters_x_);
    y = calculateNoise(m_parameters_y_);
    yaw = calculateNoise(m_parameters_yaw_);

}

/**
 * @brief Update the time
*/
void ClassRandomSinNoiseGenerator::updateTime()
{
    m_time_ = ClassHawaSimTools::getTimeSecond();
}

/**
 * @brief Check if a new start is needed
 * @return True if a new start is needed, false otherwise
*/
bool ClassRandomSinNoiseGenerator::needNewStart()
{
    if (!m_initialized_)
    {
        m_initialized_ = true;
        return true;
    }

    double dt = ClassHawaSimTools::getTimeSecond() - m_last_restart_time_;
    if (dt < 3)
        return false;

    double random = m_random_double_generator_.generate(0, 1);
    if (random < 0.004)
        return true;
    
    return false;
}

/**
 * @brief Generate new parameters for the sin noise
 * @param parameters The parameters of the sin noise
*/
void ClassRandomSinNoiseGenerator::generateNewParameters(SinNoiseParameters &parameters)
{   
    m_last_restart_time_ = ClassHawaSimTools::getTimeSecond();

    if (parameters.type == 0 || parameters.type == 1)
    {
        parameters.amplitude = m_random_double_generator_.generate(0.01, 0.15);
        parameters.frequency = m_random_double_generator_.generate(0.03, 0.1);
        parameters.phase = m_random_double_generator_.generate(0, 2 * M_PI);
        parameters.offset = m_random_double_generator_.generate(-0.15, 0.15);
    }
    else if (parameters.type == 2)
    {
        parameters.amplitude = m_random_double_generator_.generate(0.00, 0.05);
        parameters.frequency = m_random_double_generator_.generate(0.01, 0.07);
        parameters.phase = m_random_double_generator_.generate(0, 2 * M_PI);
        parameters.offset = m_random_double_generator_.generate(-0.06, 0.06);
    }
    else
    {
        std::cerr << "Error: Invalid type of sin noise" << std::endl;
        parameters.amplitude = 0;
        parameters.offset = 0;
    }
    
}

/**
 * @brief Calculate the noise value based on the given parameters
 * @param parameters The parameters of the sin noise.
 * @return The noise value
*/
double ClassRandomSinNoiseGenerator::calculateNoise(const SinNoiseParameters &parameters)
{
    return parameters.amplitude * sin(2.0 * M_PI * parameters.frequency * m_time_ + parameters.phase) + parameters.offset;
}


}