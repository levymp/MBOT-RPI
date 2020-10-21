#include <planning/astar.hpp>
#include <planning/priority.hpp>
#include <iostream>


int append_node(std::vector<Grid_Astar*> &stored_nodes)
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
        dx = future_pose.x - current_pos.x;
        dy = future_pose.y - current_pos.y;
        current_pose.theta = atan2(dy, dx);

        // append pose to path
        if(node -> parent != NULL){
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


std::vector<int> get_neighbors(Grid_Astar* cur_node, std::vector<Grid_Astar> &stored_nodes, std::vector<Grid_Astar*> &visit_q)
{
    // get current x and y position
    int x = cur_node -> cell_pos.x;
    int y = cur_node -> cell_pos.y;

    // initialize vector of neighbor indexes (index for object in stored nodes)
    std::vector<Grid_Astar*> neighbors;
    int i, j, k; 
    
    Point<int> neighbor_position;

    // go through eight neighbors 
    for(i= -1; i <= 1; i++) 
    {
        for(j= -1; j <= 1; j++ )
        {
            // get neighbor position
            neighbor_position = Point<int>(x + i, y + j);

            // check if at 0/0 (no assignment needed for this case)
            // don't do anything if cell isn't in grid
            if(!i && !j || !isCellInGrid(neighbor_position)){
                continue;
            }

            // go through all stored_nodes again
            // reset iterator count
            k = 0;
            // check if already in stored nodes (iterate through all values)
            for(std::vector<Grid_Astar>::iterator it = stored_nodes.begin(); it != stored_nodes.end(); ++it)
            {
                // check if grid position has already been used and it hasn't been visited
                // if it has been visited we don't need to go through it again
                if(*it->cell_pos.x == neighbor_position.x && *it->cell_pos.y == neighbor_position.y ){
                    if(!*it -> visited){
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
    
    /
    // setup stored_nodes
    std::vector<Grid_Astar> stored_nodes;
    
    // setup visit queue
    std::vector<Grid_Astar*> visit_q;
    
    // add start/end to list of points
    int start = append_node(stored_nodes);
    int end = append_node(stored_nodes);
    
    // set distance to start and cell_pos
    stored_nodes[start].distance = 0; 
    stored_nodes[start].cell_pos = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    stored_nodes[start].in_visit_queue = true;
    // set goal node
    stored_nodes[end].cell_pos = global_position_to_grid_cell(Point<double>(end.x, end.y), distances)

    // add start position to the empty priority queue
    visit_q.push_back(&stored_nodes[i]);

    // while visit queue not empty continue going through all nodes
    while(!visit_q.empty())
    {
        // dequeue min heap
        Grid_Astar* cur_node = dequeue(visit_q);

        // check if we're at the goal
        if(cur_node -> cell_pos == stored_nodes[end].cell_pos){
            return make_path(start, goal, cur_node);
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
        float new_dist;
        for(std::vector<int>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
        {
            // find new distance to start
            new_dist = cur_node -> distance + euc_distance(cur_node -> cell_pos, *neighbor -> cell_pos);
            
            // if new distance is less than currently assigned distance
            if(*neighbor -> distance > new_dist){
                // assign new parent
                *neighbor -> parent = cur_node; 
                // assign new distance
                *neighbor -> distance = new_dist;
                // assign priority
                // initially function of distance to start and end
                *neighbor -> prioirty = new_dist + euc_distance(stored_nodes[end].cell_pos, *neighbor -> cell_pos);
            }
            if(*neighbor -> in_visit_queue){
                // reheap because priority for node has changed
                heap(visit_q);
            }else{
                // put in visit queue
                *neighbor -> in_visit_queue = true;
                enqueue(visit_q, neighbor);
            }
        }
    }
    // did not find a valid path!
    // returning same pose 
    printf(stderr, "COULD NOT FIND CORRECT PATH");
    robot_path_t path;
    // append start to first element of path
    path.path.push_back(start);
    path.utime = start.utime;
    return path;
}
