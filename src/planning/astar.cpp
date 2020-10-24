#include <planning/astar.hpp>
#include <iostream>
#include <planning/obstacle_distance_grid.hpp>

int append_node(std::vector<Grid_Astar> &stored_nodes)
{
    // return an index and add the value to the stored_nodes 
    int i = (int) stored_nodes.size();
    stored_nodes.push_back(Grid_Astar());
    return i;
}


float euc_distance(Point<int> p1, Point<int> p2)
{
    // calculate euclidean distance to a node
    float dx, dy;
    dx = pow(((double)p2.x - p1.x), 2);
    dy = pow(((double)p2.y - p1.y), 2);
    return sqrt(dx + dy);
}


robot_path_t makepath(pose_xyt_t start, pose_xyt_t goal, Grid_Astar* node, const ObstacleDistanceGrid& distances)
{
    // initialize path and current/future pose
    robot_path_t path;
    pose_xyt_t current_pose, future_pose;
    float dx, dy;

    // set time 
    path.utime = start.utime;

    // insert goal
    path.path.push_back(goal);
    future_pose = goal;

    Point<double> global_position;
    
    // initialize path length
    path.path_length = 0;
    
    // go through all nodes
    while(node != nullptr)
    {
        // get global position
        global_position = grid_position_to_global_position((Point<double>)(node -> cell_pos), distances);
        
        // write to current pose
        current_pose.x = global_position.x;
        current_pose.y = global_position.y;

        // calculate theta
        dx = future_pose.x - current_pose.x;
        dy = future_pose.y - current_pose.y;
        current_pose.theta = atan2(dy, dx);
        
        // append pose to current_pose path (put it at the beginning)
        if((node -> parent)){
            // std::cout << current_pose.x << '\t' << current_pose.y << '\t' << current_pose.theta << '\t' << node -> priority << '\t' << node -> visited << '\t'<< node -> distance <<std::endl;
            path.path.insert(path.path.begin(), current_pose);
            path.path_length++;  
        }

        // set future pose to current pose
        future_pose = current_pose;

        // go to next node 
        node = node -> parent;
        // increase path vector length
        path.path_length++;
    }
    // append start to first position
    path.path.insert(path.path.begin(), start);
    path.path_length++;
    return path;
}


std::vector<Grid_Astar*> get_neighbors(Grid_Astar* cur_node, std::vector<Grid_Astar> &stored_nodes, std::vector<Grid_Astar*> &visit_q, const ObstacleDistanceGrid& distances, const SearchParams& params)
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
            // don't do anything if the cell is an obstacle
            if((!i && !j) || !distances.isCellInGrid(neighbor_position.x, neighbor_position.y)){
                continue;
            }else if(distances(neighbor_position.x, neighbor_position.y) < params.minDistanceToObstacle){
                continue;
            }

            // go through all stored_nodes and get index of neighbor node
            // reset iterator count
            k = 0;
            // check if already in stored nodes (iterate through all values)
            for(it= stored_nodes.begin(); it != stored_nodes.end(); ++it)
            {
                // check if grid position has already been used and it hasn't been visited
                // if it has been visited we don't need to go through it again
                if(it->cell_pos.x == neighbor_position.x && it->cell_pos.y == neighbor_position.y ){
                    if(!(it -> visited)){
                        // append to neighbors (not visited)
                        break;
                    }else{
                        // don't append to neighbors (already been visited)
                        // std::cout << "FOUND SOMETHING THAT HAS ALREADY BEEN VISITED\n";
                        k = -100;
                        break;
                    }   
                }
                // append counter
                k++;
            }

            // assign grid position of neighbor
            // if iterator is not equal to end then just add pointer to the pre-existing value
            if(it != stored_nodes.end()){
                neighbors.push_back(&(stored_nodes[k]));
            }
            // ensure that neighbor hasn't been visited before besides that add a new node (we didn't find it in stored_nodes)
            else if(k != -100){
                // append a new Grid_Astar object to stored_nodes
                int node_index = append_node(stored_nodes);
                // write the position
                stored_nodes[node_index].cell_pos = neighbor_position;
                // add it to neighbors list
                neighbors.push_back(&(stored_nodes[node_index]));
            }
        }
    }
    // return list of pointers to neighbors
    return neighbors;
}


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    // std::cout << "START POSITION: " << start << std::endl; 
    // std::cout << "END POSITION: " << end << std::endl;
   

    // setup stored_nodes
    std::vector<Grid_Astar> stored_nodes;
    // reserve 10x the memory needed
    stored_nodes.reserve(10*distances.heightInCells() * distances.widthInCells());

    // init start/goal/loop position
    Point<int> start_pos, goal_pos, cur_pos;

    // find start and goal position
    start_pos = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    goal_pos = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);


    // start and end flag
    bool start_flg = false; 
    bool goal_flg = false; 

    // indicies, start/goal index
    int iind, jind, node_idx, start_idx, goal_idx;
    
    // stored_nodes indexes
    node_idx = 0;
    for(iind= 0; iind < distances.widthInCells(); iind++){
        for(jind= 0; jind < distances.heightInCells(); jind++){
            // write current position
            cur_pos = Point<int>(iind, jind);
            
            // append to stored nodes
            stored_nodes.push_back(Grid_Astar(cur_pos));

            // check if end or goal
            if(cur_pos == start_pos){
                // assign start values
                // assign distance
                start_flg = true; 
                stored_nodes[node_idx].distance = 0;
                stored_nodes[node_idx].parent = nullptr;
                stored_nodes[node_idx].priority = euc_distance(start_pos, goal_pos);                
                start_idx = node_idx;
            }else if(cur_pos == goal_pos){
                // found goal, mark it's index
                goal_flg = true;
                goal_idx = node_idx;
            }
            // append index
            node_idx++;
        }
    }

    // setup visit queue
    std::vector<Grid_Astar*> visit_q;
    // reserve 10x memory
    visit_q.reserve(10*distances.heightInCells()*distances.widthInCells());

    // check we found start and goal is in grid
    // check that start/goal aren't in an obstacle or within minimum distance to obstacle
    if(!start_flg || !goal_flg ||
    !distances.isCellInGrid(stored_nodes[start_idx].cell_pos.x, stored_nodes[start_idx].cell_pos.y) || 
    !distances.isCellInGrid(stored_nodes[goal_idx].cell_pos.x, stored_nodes[goal_idx].cell_pos.y) ||
    distances(start_pos.x, start_pos.y) < params.minDistanceToObstacle ||
    distances(goal_pos.x, goal_pos.y) < params.minDistanceToObstacle){
        // return a path with just the start
        std::cout << "START/GOAL NOT IN GRID" << std::endl; 
        robot_path_t path;
        // append start to first element of path
        path.path.push_back(start);
        path.utime = start.utime;
        path.path_length = 1;
        return path;
    }else{
        // add start position to the empty priority queue
        visit_q.push_back(&stored_nodes[start_idx]);    
        stored_nodes[start_idx].in_visit_queue = true;  
    }

    
    
    // setup neighbors vector (pointer to stored_nodes)
    std::vector<Grid_Astar*> neighbors;
    
    // iterator for neighbors list
    std::vector<Grid_Astar*>::iterator neighbor;

    // current node (also pointer to stored_nodes)
    Grid_Astar* cur_node; 

    // distance float
    float new_dist, dist_obstacle;

    // set the current node to start node because we know it will be first value popped
    cur_node = &stored_nodes[start_idx];
    
    // while visit queue not empty and current node not goal continue going through all nodes
    while(!visit_q.empty() && cur_node -> cell_pos != stored_nodes[goal_idx].cell_pos)
    {
        // dequeue min heap
        cur_node = dequeue(visit_q);

        // mark as not in visit queue
        cur_node -> in_visit_queue = false;

        // check if node has been visited before (if it has then we don't need to do anything further)
        if(cur_node -> visited){
            continue;
        }else{
            // mark as visited
            cur_node -> visited = true;
        }

        // get neighbors to the current node (at most 8)
        // this will return less nodes if the neighbors are obstacles
        // will not return neighbors that have already been visited
        neighbors = get_neighbors(cur_node, stored_nodes, visit_q, distances, params);

        // loop through all neighbors
        for(neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
        {
            // find new distance to start
            new_dist = cur_node -> distance + euc_distance(cur_node -> cell_pos, (*neighbor)-> cell_pos);

            // if new distance calculated is less than currently assigned distance 
            // reassign neighbors parent to current node and re-calc priority
            if((*neighbor)->distance > new_dist){
                // assign new parent
                (*neighbor)->parent = cur_node;
                                
                // assign new distance
                (*neighbor)->distance = new_dist;

                // assign priority
                // add combination of dist to start and dist to finish
                (*neighbor)->priority = new_dist + euc_distance(stored_nodes[goal_idx].cell_pos, (*neighbor)-> cell_pos);

                // add heuristic on distance to obstacle
                dist_obstacle = distances((*neighbor)->cell_pos.x, (*neighbor)->cell_pos.y);

                if(dist_obstacle <= 10*params.minDistanceToObstacle){
                    (*neighbor)->priority += pow(1/dist_obstacle, params.distanceCostExponent);
                }else{
                    (*neighbor)->priority += params.maxDistanceWithCost;
                }
                
                // check if neighbor is in the visit queue already
                // if it is then reheap the visit queue (it had it's priority updated)
                if((*neighbor)->in_visit_queue && !visit_q.empty()){
                    // reheap because priority for node has changed
                    std::make_heap(visit_q.begin(), visit_q.end(), compare_priority());
                }
            }

            // put neighbor in visit queue if it hasn't already been put in
            if(!(*neighbor)->in_visit_queue){
                // put in visit queue
                (*neighbor)->in_visit_queue = true;
                enqueue(visit_q, (*neighbor));
            }
        }
        // clear out neighbors vector
        neighbors.clear();
    }

    // while loop was exited because we either found a path or no path
    // check if we're at the goal
    if(cur_node -> cell_pos == stored_nodes[goal_idx].cell_pos){
        return makepath(start, goal, cur_node, distances);
    }else{
        // did not find a valid path!
        // returning start pose 
        std::cout << "COULD NOT FIND CORRECT PATH" << std::endl;
        robot_path_t path;
        // append start to first element of path
        path.path.push_back(start);
        path.utime = start.utime;
        return path;
    }
}
