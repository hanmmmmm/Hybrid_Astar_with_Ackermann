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

#include "common/class_elemental_path2d.h"

namespace hawa
{


ClassPath2DSegment ClassPath2D::at(int index, bool &valid)
{
    if (index >= 0 && index < path_.size())
    {
        valid = true;
        return path_[index];
    }
    else{
        valid = false;
        return ClassPath2DSegment();
    }
}

size_t ClassPath2D::size()
{
    return path_.size();
}

ClassPath2DSegment ClassPath2D::get_segment(int index)
{
    return path_[index];
}

void ClassPath2D::add_segment(ClassPath2DSegment seg)
{
    path_.push_back(seg);
}

} // namespace hawa