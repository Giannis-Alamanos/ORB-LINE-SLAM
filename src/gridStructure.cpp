/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
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

#include <iostream>
#include "gridStructure.h"

//STL
#include <algorithm>
#include <stdexcept>

#include "LineIterator.h"

namespace ORB_SLAM3 {

void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords) {
    line_coords.clear();

    LineIterator it(x1, y1, x2, y2);

    std::pair<int, int> p;
    while (it.getNext(p))
        line_coords.push_back(p);
}

GridStructure::GridStructure()
{}

GridStructure::GridStructure(int rows, int cols)
    : rows(rows), cols(cols) {

    if (rows <= 0 || cols <= 0)
        throw std::runtime_error("[GridStructure] invalid dimension");

    grid.resize(cols, std::vector<std::list<int>>(rows));
}

GridStructure::~GridStructure() {

}

std::list<int>& GridStructure::at(int x, int y) {

    if (x >= 0 && x < cols &&
            y >= 0 && y < rows)
        return grid[x][y];
    else
        return out_of_bounds;
}

void GridStructure::get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const {

    int min_x = std::max(0, x - w.width.first);
    int max_x = std::min(cols, x + w.width.second + 1);

    int min_y = std::max(0, y - w.height.first);
    int max_y = std::min(rows, y + w.height.second + 1);

    for (int x_ = min_x; x_ < max_x; ++x_)
        for (int y_ = min_y; y_ < max_y; ++y_)
            indices.insert(grid[x_][y_].begin(), grid[x_][y_].end());
}

void GridStructure::clear() {

    for (int x = 0; x < cols; ++x)
        for (int y = 0; y < rows; ++y)
            grid[x][y].clear();
}

} //namesapce ORB_SLAM3
