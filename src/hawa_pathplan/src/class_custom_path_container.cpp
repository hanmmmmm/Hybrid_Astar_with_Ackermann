
#include "class_custom_path_container.h"

namespace hawa
{

/**
 * @brief Copy the path points from given reference. 
 * @param ref The coming reference.
*/
void ClassCustomPathContainer::copyFrom(ClassCustomPathContainer& ref)
{
    clear_points();
    for(auto pt : ref.getPath())
    {
        m_path_.push_back(pt);
    }
}

/**
 * @brief To know that this path has how many points. 
*/
size_t ClassCustomPathContainer::numberOfPoints()
{
    return m_path_.size();
}

/**
 * @brief Remove all the points in the path. 
*/
void ClassCustomPathContainer::clear_points()
{
    if (m_path_.size())
    {
        m_path_.clear();
    }
}

/**
 * @brief Add a new pose at the beginning of the existing path.
*/
void ClassCustomPathContainer::pushfront(std::array<double, 3> pt)
{
    m_path_.push_front(pt);
}

/**
 * @brief Add a new pose at the end of the existing path.
*/
void ClassCustomPathContainer::pushback(std::array<double, 3> pt)
{
    m_path_.push_back(pt);
}

/**
 * @brief Get the entire path. 
*/
std::deque< std::array<double, 3> >  ClassCustomPathContainer::getPath()
{
    return m_path_;
}

/**
 * @brief Get the pose at the given index.
*/
std::array<double, 3> ClassCustomPathContainer::getAt(int index)
{
    if (index < 0 ||  (index >= int(m_path_.size())) )
    {
        std::cerr << "Check input for ClassCustomPathContainer::getAt()  " 
        << index << "  pathsize " << m_path_.size() << std::endl;
        return std::array<double, 3>{0,0,0};
    }
    return m_path_[index];
}



}

