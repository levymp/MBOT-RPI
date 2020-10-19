#ifndef PRIORITY_ASTAR_HPP
#define PRIORITY_ASTAR_HPP

#include <functional>
#include <algorithm>
#include <vector>
#include <iostream>
#include <planning/astar.hpp>


struct Grid_Astar
{
    Point<double> grid_position;    //< The location in grid position
    float distance;                 //< The distance to the start via path through parent
    float priority;                 //< priority of the node based on fscore
    bool visited;                   //< flag for whether node has been visited
    Grid_Astar* parent;                 //< Parent
    Grid_Astar(): grid_position(0,0), distance(10000000), priority(-5), visited(false), parent(NULL){} //< initialization
};


struct compare_priority {
    bool operator() (const Grid_Astar* lhs,const Grid_Astar* rhs)
    {
        return ((lhs -> priority) >= (rhs -> priority));
    }
};



void print_struct(Grid_Astar Grid_Astar_value);
Grid_Astar* dequeue(std::vector<Grid_Astar*> &visit_q );
void enqueue(std::vector<Grid_Astar*> &visit_q, Grid_Astar* new_grid);


#endif // PRIORITY_ASTAR_HPP
