/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2016-2018, Ruben Gomez-Ojeda, University of Malaga.
* Copyright (C) 2016-2018, David Zuñiga-Noël, University of Malaga.         
* Copyright (C) 2016-2018, MAPIR group, University of Malaga.    
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "LineIterator.h"

//STL
#include <cmath>
#include <utility>

//Implementation of Bresenham's line drawing Algorithm
//Adapted from: https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B

namespace ORB_SLAM3 {

LineIterator::LineIterator(const double x1_, const double y1_, const double x2_, const double y2_)
    : x1(x1_), y1(y1_), x2(x2_), y2(y2_), steep(std::abs(y2_ - y1_) > std::abs(x2_ - x1_)) {

    if (steep) {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    dx = x2 - x1;
    dy = std::abs(y2 - y1);

    error = dx / 2.0;
    ystep = (y1 < y2) ? 1 : -1;

    x = static_cast<int>(x1);
    y = static_cast<int>(y1);

    maxX = static_cast<int>(x2);
}

bool LineIterator::getNext(std::pair<int, int> &pixel) {

    if (x > maxX)
        return false;

    if (steep)
        pixel = std::make_pair(y, x);
    else
        pixel = std::make_pair(x, y);

    error -= dy;
    if (error < 0) {
        y += ystep;
        error += dx;
    }

    x++;
    return true;
}

} // namespace ORB_SLAM2
