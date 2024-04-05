
#include "utils/class_utils__timestamp.h"

namespace hawa
{


ClassHawaTimeStamp::ClassHawaTimeStamp()
{
    m_stamp_ = 0;
}

ClassHawaTimeStamp::~ClassHawaTimeStamp()
{
}

void ClassHawaTimeStamp::stampNow()
{
    m_stamp_ = ClassHawaTimer::getTimeSecs();
}


bool ClassHawaTimeStamp::checkPass(const double duration_sec)
{
    return (ClassHawaTimer::getTimeSecs() - m_stamp_) > duration_sec;
}



}

