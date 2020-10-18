#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    // initialize
    // path to be returned
    robot_path_t path;
    path.utime = start.utime;
    
    // append start to first element of path
    // path.path.push_back(start);

    // add start to list of points
    visited.push_back(Grid_Astar())

    // add that value to the priority queue


    // we will have one priority struct
    // visit queue min heap
    // visited is a list of objects 
        // objects contain 
            // grid point (point int)
            // distance
            // f_score (float)
            // parent (index at list to value)

    // convert start/goal to object


    // create visit queue with start in it (with f_cost)
    // min heap

    // while visit queue not empty
        // cur_node = dequeue visit_queue

        // if cur_node == goal 
            // return path
        // visited == true for that node 

        // go through list of all neighbors
            // if in visit queue
                // calc new cost 
                // if new_cost < cur_cost:
                    // reassign cost
            // else
                // add to visit_queue (create object)
                // calculate fscore



    path.path.push_back()

    



    path.path_length = path.path.size();
    return path;
}


float distance()