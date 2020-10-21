#include <planning/astar.hpp>
#include <iostream>
#include <planning/obstacle_distance_grid.hpp>

int append_node(std::vector<Grid_Astar> &stored_nodes)
{
    int i = (int) stored_nodes.size();
    stored_nodes.push_back(Grid_Astar());
    return i; 
}


float euc_distance(Point<int> p1, Point<int> p2)
{
    float dx, dy;
    dx = pow((p2.x - p1.x), 2);
    dy = pow((p2.y - p1.y), 2);
    return sqrt(dx + dy);
}


robot_path_t makepath(pose_xyt_t start, pose_xyt_t goal, Grid_Astar* node){
    // initialize path and current/future pose
    robot_path_t path;
    pose_xyt_t current_pose, future_pose;
    float dx, dy;

    // set time 
    path.utime = start.utime;

    // insert goal
    path.path.push_back(goal);
    future_pose = goal;
    
    // go through all nodes
    while(node != NULL)
    {
        
        // create current pose
        current_pose.x = node -> cell_pos.x;
        current_pose.y = node -> cell_pos.y;
        
        // calculate theta
        dx = future_pose.x - current_pose.x;
        dy = future_pose.y - current_pose.y;
        current_pose.theta = atan2(dy, dx);
        std::cout << "WRITING POSE" << std::endl;
        // append pose to path
        if(!(node -> parent)){
            std::cout << "FOUND START! " << std::endl;
            path.path.insert(path.path.begin(), current_pose);
        }
        // set future to current
        future_pose = current_pose;

        // go to next node
        node = node -> parent;
    }
    path.path.insert(path.path.begin(), start);
    return path;
}


std::vector<Grid_Astar*> get_neighbors(Grid_Astar* cur_node, std::vector<Grid_Astar> &stored_nodes, std::vector<Grid_Astar*> &visit_q) //, const ObstacleDistanceGrid& distances)
{
    // get current x and y position
    int x = cur_node -> cell_pos.x;
    int y = cur_node -> cell_pos.y;
    
    // initialize vector of neighbor indexes (index for object in stored nodes)
    std::vector<Grid_Astar*> neighbors;
    int i, j, k; 

    Point<int> neighbor_position;

    std::vector<Grid_Astar>::iterator it;
    // go through eight neighbors 
    for(i= -1; i <= 1; i++) 
    {
        for(j= -1; j <= 1; j++ )
        {
            // get neighbor position
            neighbor_position = Point<int>(x + i, y + j);

            // check if at 0/0 (no assignment needed for this case)
            // don't do anything if cell isn't in grid
            
            // ADD IN LATER
            // !distances.isCellInGrid(neighbor_position.x, neighbor_position.y)
            if((!i && !j)){
                continue;
            }

            // go through all stored_nodes again
            // reset iterator count
            k = 0;
            // check if already in stored nodes (iterate through all values)
            
            for(it= stored_nodes.begin(); it != stored_nodes.end(); ++it)
            {
                // check if grid position has already been used and it hasn't been visited
                // if it has been visited we don't need to go through it again
                if(it->cell_pos.x == neighbor_position.x && it->cell_pos.y == neighbor_position.y ){
                    if(!it -> visited){
                        // append to neighbors (not visited)
                        break;
                    }else{
                        // don't append to neighbors (already been visited)
                        continue;
                    }
                    
                }
                // append counter
                k++;
            }

            // assign grid position of neighbor
            // if iterator is not equal to end then just add pointer to 
            if(it != stored_nodes.end()){
                neighbors.push_back(&stored_nodes[k]);
            }
            else{
                // append a new Grid_star object to stored
                int node_index = append_node(stored_nodes);
                // write the position
                stored_nodes[node_index].cell_pos = neighbor_position;
                // add it to neighbors list
                neighbors.push_back(&stored_nodes[node_index]);
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

    // setup stored_nodes
    std::vector<Grid_Astar> stored_nodes;
    
    // setup visit queue
    std::vector<Grid_Astar*> visit_q;
    
    // add start/end to list of points
    int start_idx, goal_idx; 
    start_idx = append_node(stored_nodes);
    goal_idx = append_node(stored_nodes);
    
    // set distance to start and cell_pos
    stored_nodes[start_idx].distance = 0; 
    stored_nodes[start_idx].cell_pos = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    stored_nodes[start_idx].in_visit_queue = true;
    stored_nodes[start_idx].parent = nullptr;
    // set goal node
    stored_nodes[goal_idx].cell_pos = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);

    // check if start/goal are in the grid
    if(!distances.isCellInGrid(stored_nodes[start_idx].cell_pos.x, stored_nodes[start_idx].cell_pos.y) || !distances.isCellInGrid(stored_nodes[goal_idx].cell_pos.x, stored_nodes[goal_idx].cell_pos.y)){
        std::cout << "START/GOAL NOT IN GRID" << std::endl; 
        return makepath(start, goal, &stored_nodes[start_idx]);
    }

    // add start position to the empty priority queue
    std::cout << "----ADDING START!----" << std::endl;
    visit_q.push_back(&stored_nodes[start_idx]);
    // while visit queue not empty continue going through all nodes
    std::vector<Grid_Astar*> neighbors;
    Grid_Astar* cur_node;
    while(!visit_q.empty())
    {
        // dequeue min heap
        cur_node = dequeue(visit_q);
        // check if we're at the goal
        if(cur_node -> cell_pos == stored_nodes[goal_idx].cell_pos){
            std::cout << "*******---MAKING PATH!---*******" << std::endl;
            return makepath(start, goal, cur_node);
        }

        // check if node has been visited before (if it has then we don't need to do anything further)
        if(cur_node -> visited){
            continue;
        }else{
            // mark as visited
            cur_node -> visited = true;
        }

        // get neighbors 
        neighbors = get_neighbors(cur_node, stored_nodes, visit_q, distances);
        float new_dist;
        std::cout << "-----GETTING NEIGHBORS-----" << std::endl;
        for(std::vector<Grid_Astar*>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
        {
            // find new distance to start
            new_dist = cur_node -> distance + euc_distance(cur_node -> cell_pos, (*neighbor)-> cell_pos);
            
            // if new distance is less than currently assigned distance
            if((*neighbor)->distance > new_dist){
                // assign new parent
                (*neighbor)->parent = cur_node; 
                std::cout << (*neighbor)->parent->distance << " | " <<cur_node->distance << std::endl;
                // assign new distance
                (*neighbor)->distance = new_dist;
                // assign priority
                // initially function of distance to start and end
                (*neighbor)->priority = new_dist + euc_distance(stored_nodes[goal_idx].cell_pos, (*neighbor)-> cell_pos);
            }
            if((*neighbor)->in_visit_queue){
                if(!visit_q.empty()){
                    // reheap because priority for node has changed
                    
                    // std::cout << "<---------- REHEAPING ---------->   SIZE " << visit_q.size() << std::endl;
                    // heap(visit_q);
                    std::make_heap(visit_q.begin(), visit_q.end(), compare_priority());
                }
            }else{
                // put in visit queue
                // std::cout <<     "<----- ADDING TO VISIT QUEUE -----> SIZE " << visit_q.size() << std::endl;
                (*neighbor)->in_visit_queue = true;
                enqueue(visit_q, (*neighbor));
            }
        }
        neighbors.clear();
    }
    // did not find a valid path!
    // returning same pose 
    std::cout << "COULD NOT FIND CORRECT PATH" << std::endl;
    robot_path_t path;
    // append start to first element of path
    path.path.push_back(start);
    path.utime = start.utime;
    return path;
}
