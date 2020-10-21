#ifndef PRIORITY_ASTAR_HPP
#define PRIORITY_ASTAR_HPP

#include <functional>
#include <algorithm>
#include <vector>
#include <iostream>
#include <common/point.hpp>
// #include <planning/astar.hpp>


struct Grid_Astar
{
    Point<double> cell_pos;    //< The location in grid position
    float distance;                 //< The distance to the start via path through parent
    float priority;                 //< priority of the node based on fscore
    bool visited;                   //< flag for whether node has been visited
    bool in_visit_queue;             //< flag for whether node is in the visit queue
    Grid_Astar* parent;                 //< Parent
    Grid_Astar(): cell_pos(0,0), distance(10000000), priority(-5), visited(false), in_visit_queue(false), parent(NULL){} //< initialization
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
void heap(std::vector<Grid_Astar*> &visit_q);

#endif // PRIORITY_ASTAR_HPP
