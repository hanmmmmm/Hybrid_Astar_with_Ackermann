

#include "class_tool_general.h"

namespace hawa
{


/**
 * @brief Calculate the occupancy value based on the distance from the center. 
 * @param vmax The maximum occupancy value. 
 * @param vmin The minimum occupancy value. 
 * @param pos_x The x position. 
 * @param pos_y The y position. 
 * @return The occupancy value. 
*/
int8_t classToolGeneral::calcOccValue(const int8_t vmax, const int8_t vmin, const int pos_x, const int pos_y, const int radius, const int width)
{
    if (pos_x < 0 || pos_y < 0 || pos_x > width-1 || pos_y > width-1)
    {
        std::cerr << "calcOccValue() invalid pos_x:" << pos_x << " pos_y:" << pos_y << std::endl;
        return 0;
    }

    int8_t dx = radius - pos_x;
    int8_t dy = radius - pos_y;
    float dist = std::sqrt(dx*dx + dy*dy);
    float ratio = dist / radius;
    int8_t val = vmax - (vmax - vmin) * ratio;
    return val;
}


} // namespace hawa

