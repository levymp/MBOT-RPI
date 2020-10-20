#include <planning/priority.hpp>


void print_struct(Grid_Astar Grid_Astar_value){
    std::cout << "DISTANCE: "<< Grid_Astar_value.distance << std::endl;
    std::cout << "PRIORITY: "<< Grid_Astar_value.priority << std::endl;
    std::cout << "VISITED: "<< Grid_Astar_value.visited << std::endl;
    std::cout << "POINT: "<< Grid_Astar_value.grid_position << std::endl;
    std::cout << "PARENT: "<< Grid_Astar_value.parent << std::endl;
    
}

// assumes that the visit_q is already a min heap
Grid_Astar* dequeue(std::vector<Grid_Astar*> &visit_q )
{
    // move min to the back
    std::pop_heap(visit_q.begin(), visit_q.end(), compare_priority());
    // grab end value (min)
    Grid_Astar* min = visit_q.back();
    // remove end value
    visit_q.pop_back();

    // return min and visit_q is still a heap
    return min;
}


// assuming you already have a min heap
void enqueue(std::vector<Grid_Astar*> &visit_q, Grid_Astar* new_grid)
{
    // add value to the end
    visit_q.push_back(new_grid);
    
    // reheap
    std::push_heap(visit_q.begin(), visit_q.end(), compare_priority());
}

void heap(std::vector<Grid_Astar*> &visit_q)
{
    std::make_heap(visit_q.begin(), visit_q.end(), compare_priority());
}
