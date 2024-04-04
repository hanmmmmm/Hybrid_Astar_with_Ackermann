
#include "class_inflation_samples.h"


namespace hawa
{

ClassInflationSamples::ClassInflationSamples(const int min_occ_val, const int radius)
: m_min_occ_val_(min_occ_val), m_radius_(radius)
{
    if (m_radius_ < 1)
    {
        int rd = 4;
        std::cerr << "ClassInflationSamples() invalid radius:" << radius << ". Settinng it as " << rd << std::endl;
        m_radius_ = rd;
    }
    if (m_min_occ_val_ < 0 || m_min_occ_val_ > 100)
    {
        int occ = 70;
        std::cerr << "ClassInflationSamples() invalid min_occ_val:" << min_occ_val << ". Settinng it as " << occ << std::endl;
        m_min_occ_val_ = occ;
    }
    m_width_ = m_radius_ * 2 + 1;

    fillSample();

    printSample();
}

ClassInflationSamples::~ClassInflationSamples()
{
}

// void ClassInflationSamples::calcTheStep(const int8_t vmax, const int8_t vmin, const int num_of_intervals, std::vector<int8_t> &v)
// {
//     if (num_of_intervals < 1)
//     {
//         std::cerr << "calcTheStep() invalid num_of_intervals:" << num_of_intervals << std::endl;
//         return;
//     }

//     if (vmax < vmin)
//     {
//         std::cerr << "calcTheStep() invalid vmax:" << vmax << " vmin:" << vmin << std::endl;
//         return;
//     }

//     int step_size = (vmax - vmin) / num_of_intervals;
    
//     for (int i = 0; i < num_of_intervals; i++)
//     {
//         v.push_back(vmax - i * step_size);
//     }
//     v.push_back(vmin);

// }


// /**
//  * @brief Calculate the occupancy value based on the distance from the center. 
//  * @param vmax The maximum occupancy value. 
//  * @param vmin The minimum occupancy value. 
//  * @param pos_x The x position. 
//  * @param pos_y The y position. 
//  * @return The occupancy value. 
// */
// int8_t ClassInflationSamples::calcOccValue(const int8_t vmax, const int8_t vmin, const int pos_x, const int pos_y, const int radius, const int width)
// {
//     if (pos_x < 0 || pos_y < 0 || pos_x > m_width_-1 || pos_y > m_width_-1)
//     {
//         std::cerr << "calcOccValue() invalid pos_x:" << pos_x << " pos_y:" << pos_y << std::endl;
//         return 0;
//     }

//     int8_t dx = m_radius_ - pos_x;
//     int8_t dy = m_radius_ - pos_y;
//     float dist = std::sqrt(dx*dx + dy*dy);
//     float ratio = dist / m_radius_;
//     int8_t val = vmax - (vmax - vmin) * ratio;
//     return val;
// }

/**
 * @brief Fill the inflation sample. 
*/
void ClassInflationSamples::fillSample()
{
    m_inflate_sample_.clear();
    m_inflate_sample_.resize(m_width_);
    for (int i = 0; i < m_width_; i++)
    {
        m_inflate_sample_[i].resize(m_width_);
    }

    for (int i = 0; i < m_width_; i++)
    {
        for (int j = 0; j < m_width_; j++)
        {
            auto val = classToolGeneral::calcOccValue(vmax, m_min_occ_val_, i, j, m_radius_, m_width_);
            if (val < m_min_occ_val_)
            {
                val = vclr;
            }
            m_inflate_sample_[i][j] = val; 
        }
    }

}

// /**
//  * @brief Set the value of the radius. The value is in grid wise. 
//  * @param rad The radius. It should be at least 1. Be careful about the maximum supported value. It can be found
//  * in the function prepareSampleByRadius().
// */
// void ClassInflationSamples::setRadius(const int rad)
// {
//     if (rad < 1)
//     {
//         std::stringstream ss;
//         ss << "setRadius() invalid radius:" << rad;
//         // RCLCPP_INFO(rclcpp::get_logger("ClassInflationSamples"), ss.str());
//         return;
//     }
//     m_radius_val_ = rad;
//     m_radius_is_set_ = true;
// }

// /**
//  * @brief Get value of the radius. 
//  * @return The radius.
// */
// int ClassInflationSamples::getRadius()
// {
//     return m_radius_;
// }

/**
 * @brief Call this function to get the inflation. 
 * @return The inflation sample. 
*/
std::vector<std::vector<int8_t>> ClassInflationSamples::getInflationSample()
{
    return m_inflate_sample_;
}

/**
 * @brief Print the inflation sample. 
*/
void ClassInflationSamples::printSample()
{
    for (int i = 0; i < m_width_; i++)
    {
        for (int j = 0; j < m_width_; j++)
        {
            std::cout << m_inflate_sample_[i][j] << " ";
        }
        std::cout << std::endl;
    }

}

// /**
//  * @brief Generate the inflation sample, and save in a member variable.
// */
// void ClassInflationSamples::prepareSampleByRadius()
// {
//     m_inflate_sample_.clear();

//     if( m_radius_val_ == 1)
//     {
//         m_inflate_sample_.push_back( {v2,v1,v2} );
//         m_inflate_sample_.push_back( {v1,v0,v1} );
//         m_inflate_sample_.push_back( {v2,v1,v2} );
//     }
//     else if( m_radius_val_ == 2)
//     {
//         m_inflate_sample_.push_back( {cr,v3,v2,v3,cr} );
//         m_inflate_sample_.push_back( {v3,v1,v1,v1,v3} );
//         m_inflate_sample_.push_back( {v2,v1,v0,v1,v2} );
//         m_inflate_sample_.push_back( {v3,v1,v1,v1,v3} );
//         m_inflate_sample_.push_back( {cr,v3,v2,v3,cr} );
//     }
//     else if( m_radius_val_ == 3)
//     {
//         m_inflate_sample_.push_back( {cr,cr,v3,v3,v3,cr,cr} );
//         m_inflate_sample_.push_back( {cr,v2,v2,v2,v2,v2,cr} );
//         m_inflate_sample_.push_back( {v3,v2,v1,v1,v1,v2,v3} );
//         m_inflate_sample_.push_back( {v3,v2,v1,v0,v1,v2,v3} );
//         m_inflate_sample_.push_back( {v3,v2,v1,v1,v1,v2,v3} );
//         m_inflate_sample_.push_back( {cr,v2,v2,v2,v2,v2,cr} );
//         m_inflate_sample_.push_back( {cr,cr,v3,v3,v3,cr,cr} );
//     }
//     else if( m_radius_val_ == 4)
//     {
//         m_inflate_sample_.push_back( {cr,cr,cr,v3,v3,v3,cr,cr,cr} );
//         m_inflate_sample_.push_back( {cr,cr,v4,v3,v3,v3,v4,cr,cr} );
//         m_inflate_sample_.push_back( {cr,v3,v2,v2,v2,v2,v2,v3,cr} );
//         m_inflate_sample_.push_back( {v4,v3,v2,v1,v1,v1,v2,v3,v4} );
//         m_inflate_sample_.push_back( {v4,v3,v2,v1,v0,v1,v2,v3,v4} );
//         m_inflate_sample_.push_back( {v4,v3,v2,v1,v1,v1,v2,v3,v4} );
//         m_inflate_sample_.push_back( {cr,v3,v2,v2,v2,v2,v2,v3,cr} );
//         m_inflate_sample_.push_back( {cr,cr,v4,v3,v3,v3,v4,cr,cr} );
//         m_inflate_sample_.push_back( {cr,cr,cr,v3,v3,v3,cr,cr,cr} );
//     }
//     else if( m_radius_val_ == 5)
//     {
//         m_inflate_sample_.push_back( {cr,cr,cr,cr,v5,v5,v5,cr,cr,cr,cr} );
//         m_inflate_sample_.push_back( {cr,cr,cr,v5,v4,v4,v4,v5,cr,cr,cr} );
//         m_inflate_sample_.push_back( {cr,cr,v5,v4,v3,v3,v3,v4,v5,cr,cr} );
//         m_inflate_sample_.push_back( {cr,v5,v3,v2,v2,v2,v2,v2,v3,v5,cr} );
//         m_inflate_sample_.push_back( {v5,v4,v3,v2,v1,v1,v1,v2,v3,v4,v5} );
//         m_inflate_sample_.push_back( {v5,v4,v3,v2,v1,v0,v1,v2,v3,v4,v5} );
//         m_inflate_sample_.push_back( {v5,v4,v3,v2,v1,v1,v1,v2,v3,v4,v5} );
//         m_inflate_sample_.push_back( {cr,v5,v3,v2,v2,v2,v2,v2,v3,v5,cr} );
//         m_inflate_sample_.push_back( {cr,cr,v5,v4,v3,v3,v3,v4,v5,cr,cr} );
//         m_inflate_sample_.push_back( {cr,cr,cr,v5,v4,v4,v4,v5,cr,cr,cr} );
//         m_inflate_sample_.push_back( {cr,cr,cr,cr,v5,v5,v5,cr,cr,cr,cr} );
//     }
//     else
//     {
//         // ROS_ERROR_STREAM("Invalid inflation size:" << m_radius_val_);
//     }
// }






} // namespace hawa

