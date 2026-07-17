#pragma once

#include <map>
#include <vector>
#include <utility>

struct Cell {
    int point_idx;
    std::vector<double> weights;

    Cell(int point_idx);
};

void precompute_angles();
