#ifndef CLASS_CUSTOM_PATH_H
#define CLASS_CUSTOM_PATH_H

#include <iostream>
#include <deque>
#include <array>

using std::deque, std::array;

class ClassCustomPathContainer
{
private:
    deque< array<double, 3> >  m_path;


public:
    ClassCustomPathContainer();
    ~ClassCustomPathContainer();
    void copy_from(ClassCustomPathContainer& ref);
    void clear_points();
    void pushfront(array<double, 3> pt);
    void pushback(array<double, 3> pt);
    deque< array<double, 3> >  get_path();
    size_t number_of_points();
    array<double, 3> get_at(int index);
};

ClassCustomPathContainer::ClassCustomPathContainer()
{
}

ClassCustomPathContainer::~ClassCustomPathContainer()
{
}

void ClassCustomPathContainer::copy_from(ClassCustomPathContainer& ref)
{
    clear_points();
    for(auto pt : ref.get_path())
    {
        m_path.push_back(pt);
    }
}

size_t ClassCustomPathContainer::number_of_points()
{
    return m_path.size();
}

void ClassCustomPathContainer::clear_points()
{
    if (m_path.size())
    {
        m_path.clear();
    }
}


void ClassCustomPathContainer::pushfront(array<double, 3> pt)
{
    m_path.push_front(pt);
}

void ClassCustomPathContainer::pushback(array<double, 3> pt)
{
    m_path.push_back(pt);
}


deque< array<double, 3> >  ClassCustomPathContainer::get_path()
{
    return m_path;
}


array<double, 3> ClassCustomPathContainer::get_at(int index)
{
    if (index < 0 ||  (index >= m_path.size()) )
    {
        std::cerr << "Check input for ClassCustomPathContainer::get_at()  " << index << "  pathsize " << m_path.size() << std::endl;
        return array<double, 3>{0,0,0};
    }
    return m_path[index];
}


#endif