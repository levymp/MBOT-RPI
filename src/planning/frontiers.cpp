#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
robot_path_t path_to_frontier(const frontier_t& frontier, 
                              const pose_xyt_t& pose, 
                              const OccupancyGrid& map,
                              const MotionPlanner& planner);
pose_xyt_t nearest_navigable_cell(pose_xyt_t pose, 
                                  Point<float> desiredPosition, 
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner);
pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner);
double path_length(const robot_path_t& path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontiers;
}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */
    unsigned int Target_frontier_id = 0;
    unsigned int Target_frontier_point_id = 0;
    float min_Dis = 1000000000000;
    float Dis = 0;

    for (size_t i = 0; i < frontiers.size(); i++) {
        for (size_t j = 0; j < frontiers[i].cells.size(); j++) {
            Point<float> temp_pt;
            temp_pt.x = frontiers[i].cells[j].x;
            temp_pt.y = frontiers[i].cells[j].y;

            Dis = sqrtf(powf(robotPose.x - temp_pt.x, 2) + powf(robotPose.y - temp_pt.y, 2));

            if (Dis < min_Dis) {
                min_Dis = Dis;
                Target_frontier_id = i;
                Target_frontier_point_id = j;
            }
        }
    }

    Point<float> Target_frontier_point;
    // Target_frontier_point.x = frontiers[Target_frontier_id].cells[Target_frontier_point_id].x;
    // Target_frontier_point.y = frontiers[Target_frontier_id].cells[Target_frontier_point_id].y;
    Target_frontier_point.x = frontiers[Target_frontier_id].cells[frontiers[Target_frontier_id].cells.size()/2].x;
    Target_frontier_point.y = frontiers[Target_frontier_id].cells[frontiers[Target_frontier_id].cells.size()/2].y;

    Point<float> Target_point;
    Point<int> Target_cell;
    bool Target_found = false;
    int num_of_angles = 72;
    float angle_increment = 2 * M_PI / num_of_angles;
    float radius_init = 0.05f; // in meters

    for (float radius = radius_init; ; radius += 0.05) {

        for (int i = 0; i < num_of_angles; i++) {
            float angle = i * angle_increment;
            Point<float> new_point;
            new_point.x = Target_frontier_point.x + radius * cosf(angle);
            new_point.y = Target_frontier_point.y + radius * sinf(angle);

            if (new_point.x >= 0 && new_point.x < map.widthInMeters()
                && new_point.y >= 0 && new_point.y < map.heightInMeters()) {
                
                float dist = 0.0f;
                dist = sqrtf(powf(robotPose.x - new_point.x, 2) + powf(robotPose.y - new_point.y, 2));
                if (dist > 0.05f) {
                    Point<int> new_cell = global_position_to_grid_cell(new_point, map);
                    if (map(new_cell.x, new_cell.y) < 0) {
                        Target_point = new_point;
                        Target_cell = new_cell;
                        Target_found = true;
                        break;
                    }
                } 
            }
        }
        if (Target_found) {
            break;
        }
    }

    pose_xyt_t Target_pose;
    Target_pose.x = Target_point.x;
    Target_pose.y = Target_point.y;

    if (map.isCellInGrid(Target_cell.x, Target_cell.y)) {
        // robot_path_t path = planner.planPath(robotPose, Target_pose);
        // return path;

        robot_path_t path;
        path.path.resize(2);
        Target_pose.theta = robotPose.theta;
        path.path[0] = robotPose;
        path.path[1] = Target_pose;
        std::cout << Target_pose.x << "   " << Target_pose.y  << std::endl;
        path.path_length = 2;
        return path;
         
    } else {
        // std::cout << "invalid goal: message from frontier.cpp" << std::endl;
        exit(1);
    }

}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}