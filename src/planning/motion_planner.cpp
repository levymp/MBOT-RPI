#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, 
                                     const pose_xyt_t& goal, 
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";        

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{   
    // std::cout << "from isValidGoal: goal.x =  " << goal.x << " goal.y = " << goal.y  << std::endl;  
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    if (goal.x != 0.0f && goal.y !=  0.0f) {
        if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) {
            // std::cout << "TARGET IS WITHIN ROBOT DIAMTER" << std::endl;
            return false;
        } 
    }
    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        // std::cout << "distances_(" << goalCell.x << " , " << goalCell.y << ") = " << distances_(goalCell.x, goalCell.y) << std::endl;
        return distances_(goalCell.x, goalCell.y) > .2;
    }
    
    // A goal must be in the map for the robot to reach it
    // std::cout << "THE GOAL IS NOT IN THE MAP" << std::endl;
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////
    // assume before this is called the obstacle distance grid is updated

    // get all of the targes
    if(path.path_length <= 1){
        return true;
    }

    // target grid position
    Point<int> grid_pos;

    // iterate through all poses
    for(auto pose : path.path){
        // get grid position of cell
        grid_pos = global_position_to_grid_cell(Point<double>(pose.x, pose.y), distances_);
        
        // check if within robot distance
        if(distances_(grid_pos.x, grid_pos.y) <= .175){
            std::cout << "PATH IS NOT SAFE NOW!\n";
            return false;
        }
    }        
    
    return true;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1.0;
}
