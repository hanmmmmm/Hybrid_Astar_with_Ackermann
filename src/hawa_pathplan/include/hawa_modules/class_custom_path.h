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
 * @file class_custom_path.h
 * @author Mingjie
 * @brief This class is for storing the path in a custom type.
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CLASS_CUSTOM_PATH_H
#define CLASS_CUSTOM_PATH_H

#include <iostream>
#include <deque>
#include <array>

class ClassCustomPathContainer
{
private:
    std::deque< std::array<double, 3> >  m_path_;

public:
    ClassCustomPathContainer();
    ~ClassCustomPathContainer();
    void copyFrom(ClassCustomPathContainer& ref);
    void clear_points();
    void pushfront(std::array<double, 3> pt);
    void pushback(std::array<double, 3> pt);
    std::deque< std::array<double, 3> >  getPath();
    size_t numberOfPoints();
    std::array<double, 3> getAt(int index);
};

ClassCustomPathContainer::ClassCustomPathContainer()
{
}

ClassCustomPathContainer::~ClassCustomPathContainer()
{
}

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
    if (index < 0 ||  (index >= m_path_.size()) )
    {
        std::cerr << "Check input for ClassCustomPathContainer::getAt()  " 
        << index << "  pathsize " << m_path_.size() << std::endl;
        return std::array<double, 3>{0,0,0};
    }
    return m_path_[index];
}


#endif