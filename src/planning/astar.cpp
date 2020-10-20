#include <planning/astar.hpp>
#include <planning/priority.hpp>
#include <iostream>


int append_node(std::vector<Grid_Astar*> &visit_q)
{
    int i = (int) visit_q.size();
    visit_q.push_back(Grid_Astar());
    return i; 
}

void get_neighbors(Grid_Astar* cur_node, std::vector<Grid_Astar> &stored_nodes, const ObstacleDistanceGrid& distances)
{
    cur_node -> grid_position

}


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
    path.path.push_back(start);

    // setup stored_nodes
    std::vector<Grid_Astar> stored_nodes;
    
    // setup visit queue
    std::vector<Grid_Astar*> visit_q;
    
    // add start/end to list of points
    int start = append_node(stored_nodes);
    int end = append_node(stored_nodes);
    
    // set distance to start and grid_position
    stored_nodes[start].distance = 0;
    stored_nodes[start].grid_position = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    // set goal node
    stored_nodes[end].grid_position = global_position_to_grid_cell(Point<double>(end.x, end.y), distances)

    // add that value to the empty priority queue
    visit_q.push_back(&stored_nodes[i]);

    // while visit queue not empty
    while(!visit_q.empty())
    {
        // dequeue min heap
        Grid_Astar* cur_node = dequeue(visit_q);
        // check if we're at the goal
        if(cur_node -> grid_position == stored_nodes[end].grid_position){
            break;
        }

        // check if node has been visited before (if it has then we don't need to do anything further)
        if(cur_node -> visited){
            continue;
        }else{
            // mark as visited
            cur_node -> visited = true;
        }
        // get neighbors 
        get_neighbors(cur_node, );
        std::vector
        for()


    }
        // go through list of all neighbors
            // if in visit queue
                // calc new cost 
                // if new_cost < cur_cost:
                    // reassign cost
            // else
                // add to visit_queue (create object)
                // calculate fscore



    // keep appending path until start value found
    // while()


    // path.path.push_back()

    



    path.path_length = path.path.size();
    return path;
}


// float distance()