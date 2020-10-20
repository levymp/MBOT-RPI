#include <planning/astar.hpp>
#include <planning/priority.hpp>
#include <iostream>


int append_node(std::vector<Grid_Astar*> &stored_nodes)
{
    int i = (int) stored_nodes.size();
    stored_nodes.push_back(Grid_Astar());
    return i; 
}

std::vector<int> get_neighbors(Grid_Astar* cur_node, std::vector<Grid_Astar> &stored_nodes)
{
    // get current x and y position
    int x = cur_node -> grid_position.x;
    int y = cur_node -> grid_position.y;

    // initialize vector of neighbor indexes (index for object in stored nodes)
    std::vector<Grid_Astar*> neighbors;
    int i, j, k; 
    bool add_new;
    Point<int> neighbor_position;

    // go through eight neighbors 
    for(i= -1; i <= 1; i++) 
    {
        for(j= -1; j <= 1; j++)
        {
            // check if at 0/0 (no assignment needed for this case)
            if(!i && !j){
                continue;
            }
            // get neighbor position
            neighbor_position = Point<int>(x + i, y + j);

            // go through all stored_nodes again
            // reset iterator and add flag
            k = 0;
            add_new = true; 
            // check if already in stored nodes
            for(std::vector<Grid_Astar>::iterator it = stored_nodes.begin(); it != stored_nodes.end(); ++it)
            {
                // check if grid position has already been used and it hasn't been visited
                // if it has been visited we don't need to go through it again
                if(*it->grid_position.x == neighbor_position.x && *it->grid_position.y == neighbor_position.y && !*it -> visited){
                    break;
                }
                k++;
            }
            // append empty grid point to stored_nodes
            
            // assign grid position of neighbor
            // if iterator is not equal to end then just add pointer to 
            if(it != stored_nodes.end()){
                neighbors.push_back(&stored_nodes[k]);
            }
            else{
                // append a new Grid_star object
                int node_index = append_node(stored_nodes);
                // write the position
                stored_nodes[node_index].grid_position = neighbor_position;
                // add it to neighbors list
                neighbors.push_back(&stored_nodes[node_index])
            }
        }
    }
    // return list of index of neighbors
    return neighbors;
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

    // add start position to the empty priority queue
    visit_q.push_back(&stored_nodes[i]);

    // while visit queue not empty continue going through all nodes
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
        std::vector<Grid_Astar*> neighbors = get_neighbors(cur_node, stored_nodes);
        
        for(std::vector<int>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
        {
            // neighbors have already been put in stored_nodes 
                        

        }
    }
        // go through list of all neighbors
            // if in visit queue
                // calc new cost 
                // if new_cost < cur_cost:
                    // reassign cost
            // else
                // add to visit_queue (create object)
                // calculate fscore

    // path.path.push_back()
    path.path_length = path.path.size();
    return path;
}
