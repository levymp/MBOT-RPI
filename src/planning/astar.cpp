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

        // std::cout << "WRITING POSE" << std::endl;
        
        // append pose to path
        if((node -> parent)){
            std::cout << current_pose.x << '\t' << current_pose.y << '\t' << current_pose.theta << '\t' << node -> priority << '\t' << node -> visited << '\t'<< node -> distance <<std::endl;
            path.path.insert(path.path.begin(), current_pose);
            path.path_length++;  
        }
        // set future to current
        future_pose = current_pose;

        // go to next node
        node = node -> parent;
        path.path_length++;
    }
    path.path.insert(path.path.begin(), start);
    path.path_length++;
    return path;
}


std::vector<Grid_Astar*> get_neighbors(Grid_Astar* cur_node, std::vector<Grid_Astar> &stored_nodes, std::vector<Grid_Astar*> &visit_q, const ObstacleDistanceGrid& distances)
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
            // 
            if((!i && !j) || !distances.isCellInGrid(neighbor_position.x, neighbor_position.y)){
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
                    if(!(it -> visited)){
                        // append to neighbors (not visited)
                        break;
                    }else{
                        // don't append to neighbors (already been visited)
                        std::cout << "FOUND SOMETHING THAT HAS ALREADY BEEN VISITED\n";
                        k = -100;
                        break;
                    }   
                }
                // append counter
                k++;
            }

            // assign grid position of neighbor
            // if iterator is not equal to end then just add pointer to current value
            if(it != stored_nodes.end()){
                
                neighbors.push_back(&(stored_nodes[k]));
            }
            // ensure that neighbor hasn't been visited before
            else if(k != -100){
                std::cout << "UH OHHHH!!! WE DIDN'T FIND IN STORED NODES!\n";
                // append a new Grid_Astar object to stored_nodes
                int node_index = append_node(stored_nodes);
                // write the position
                stored_nodes[node_index].cell_pos = neighbor_position;
                // add it to neighbors list
                neighbors.push_back(&(stored_nodes[node_index]));
            }
        }
    }
    // return list of index of neighbors
    std::cout << "----FOUND " << neighbors.size() << "NEIGHBORS ----\n"<< std::endl; 
    return neighbors;
}


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    std::cout << "START POSITION:" << std::endl;
    std::cout << start.x << '\t' << start.y << '\t' << start.theta << std::endl;
    std::cout << "END POSITION:" << std::endl;
    std::cout << goal.x << '\t' << goal.y << '\t' << goal.theta << std::endl;

    // setup stored_nodes
    std::vector<Grid_Astar> stored_nodes;
    // stored_nodes.reserve(distances.heightInCells() * distances.widthInCells());

    // find start/goal cell position 
    Point<int> start_pos, goal_pos, cur_pos;

    // find start and goal position
    start_pos = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    goal_pos = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);


    // start and end flag
    bool start_flg = false; 
    bool goal_flg = false; 

    // indicies
    int iind, jind, node_idx, start_idx, goal_idx;
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
                goal_flg = true;
                goal_idx = node_idx;

            }
            // append index
            node_idx++;
        }
    }

    // setup visit queue
    std::vector<Grid_Astar*> visit_q;
    // visit_q.reserve(distances.heightInCells()*distances.widthInCells());

    // check if start/goal are in the grid
    if(!start_flg || !goal_flg || !distances.isCellInGrid(stored_nodes[start_idx].cell_pos.x, stored_nodes[start_idx].cell_pos.y) || !distances.isCellInGrid(stored_nodes[goal_idx].cell_pos.x, stored_nodes[goal_idx].cell_pos.y)){
        std::cout << "START/GOAL NOT IN GRID" << std::endl; 
    }else{
        // add start position to the empty priority queue
        std::cout << "----ADDING START!----" << std::endl;
        visit_q.push_back(&stored_nodes[start_idx]);    
        stored_nodes[start_idx].in_visit_queue = true;  
    }

    // while visit queue not empty continue going through all nodes
    std::vector<Grid_Astar*> neighbors;
    Grid_Astar* cur_node; 
    cur_node = &stored_nodes[start_idx];
    
    Point<double> verification_point = Point<double>(-2.65,5);
    while(!visit_q.empty() && cur_node -> cell_pos != stored_nodes[goal_idx].cell_pos)
    {
        // dequeue min heap
        cur_node = dequeue(visit_q);
        std::cout << "------------CURRENT NODE:-----------\n";

        print_struct(*cur_node);



        cur_node -> in_visit_queue = false;

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
        // std::cout << "-----GETTING NEIGHBORS-----" << std::endl;
        for(std::vector<Grid_Astar*>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
        {
            std::cout << "------NEIGHBOR FOR--------- " << cur_node -> cell_pos << std::endl;
            print_struct(**neighbor);

            // find new distance to start
            new_dist = cur_node -> distance;
            new_dist += euc_distance(cur_node -> cell_pos, (*neighbor)-> cell_pos);

            // if new distance is less than currently assigned distance
            if((*neighbor)->distance > new_dist){
                // assign new parent
                (*neighbor)->parent = cur_node;
                
                // std::cout << (*neighbor)->parent->distance << " | " <<cur_node->distance << std::endl;
                
                // assign new distance
                (*neighbor)->distance = new_dist;
                // assign priority
                // initially function of distance to start and end
                (*neighbor)->priority = new_dist + euc_distance(stored_nodes[goal_idx].cell_pos, (*neighbor)-> cell_pos);

                if((*neighbor)->in_visit_queue){
                    if(!visit_q.empty()){
                        // reheap because priority for node has changed
                        std::cout << "<---------- REHEAPING ---------->   SIZE " << visit_q.size() << std::endl;
                        std::make_heap(visit_q.begin(), visit_q.end(), compare_priority());
                    }
                }
            }

            if(std::abs(distance_between_points(grid_position_to_global_position(Point<double>((*neighbor) -> cell_pos), distances), verification_point)) < 0.01){
                std::cout << "!!!!!!!!NEIGHBOR FOUND!!!!" << std::endl;
                Grid_Astar* p = (*neighbor) -> parent;
                std::cout << "!!!!!!!!Got pointer to parent \t cell pos:" << p->cell_pos << std::endl;
                Point<double> p_pos = grid_position_to_global_position(Point<double>(p->cell_pos), distances);
                std::cout << "!!!!!!!!Got parent position" << std::endl;
                std::cout << "!!!!!!!!PARENT POSITION: " << p_pos << std::endl;
            }
            // (*neighbor)->in_visit_queue = true;
            // enqueue(visit_q, (*neighbor));

            if(!(*neighbor)->in_visit_queue){
                // put in visit queue
                std::cout <<     "<----- ADDING TO VISIT QUEUE -----> SIZE " << visit_q.size() <<std::endl;
                (*neighbor)->in_visit_queue = true;
                enqueue(visit_q, (*neighbor));
            }
        }
        neighbors.clear();
    }

    std::cout << "VISIT QUEUE" <<"AFTER LOOP:\n";
    std::cout << visit_q.size() << '\t' << visit_q.max_size() << '\t' << std::endl;
    std::cout << "STORED NODES" <<"AFTER LOOP:\n";
    std::cout << stored_nodes.size() << '\t' << stored_nodes.max_size() << '\t' << std::endl;

    // check if we're at the goal
    if(cur_node -> cell_pos == stored_nodes[goal_idx].cell_pos){
        std::cout << "*******---MAKING PATH!---*******" << std::endl;
        return makepath(start, goal, cur_node, distances);
    }else{
        // did not find a valid path!
        // returning same pose 
        std::cout << "COULD NOT FIND CORRECT PATH" << std::endl;
        robot_path_t path;
        // append start to first element of path
        path.path.push_back(start);
        path.utime = start.utime;
        return path;
    }
}
