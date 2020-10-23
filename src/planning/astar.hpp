#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <planning/priority.hpp>
#include <common/grid_utils.hpp>


class ObstacleDistanceGrid;


/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};


/**
 * append_node passes stored_nodes by reference and appends a Grid_Astar object and returns that new values index
 * 
 * \param   stored_nodes    Vector of all nodes composed of Grid_Astar objects
 * \return  The index of the appended Grid_Astar object
 * 
 */
int append_node(std::vector<Grid_Astar> &stored_nodes);

/**
 * \param   p1  Point<int> of a grid cell
 * \param   p2  Point<int> of a grid cell
 * \return      The euclidean distance between the grid cells
 */
float euc_distance(Point<int> p1, Point<int> p2);
/**
 * \param   start       Starting pose of the robot
 * \param   goal        Goal pose of the robot
 * \param   node        node that has been last visited (closest to the goal). This will be used to start the recursive call for parents
 * \param   distances   Distances to the nearest obstacle for each cell in the grid (also grid test)
 * \return  The path that needs to be traveled from start to goal (if it exists). In robot_path_t spec.
 */
robot_path_t makepath(pose_xyt_t start, pose_xyt_t goal, Grid_Astar* node, const ObstacleDistanceGrid& distances);

/**
 * \param   cur_node        Current node that is being visited (pointer to value in stored_nodes)
 * \param   stored_nodes    All nodes  in the grid (of type Grid_Astar)
 * \param   visit_q         Visit Queue (min heap on priority) pointers to value in stored_nodes
 * \param   distances   Distances to the nearest obstacle for each cell in the grid (also grid test)
 * \return  Vector <Grid_Astar*> of all neighbors to cur_node in Stored_nodes (appends them if they exist in grid and aren't already in stored_nodes)
 */
std::vector<Grid_Astar*> get_neighbors(Grid_Astar* cur_node, std::vector<Grid_Astar> &stored_nodes, std::vector<Grid_Astar*> &visit_q, const ObstacleDistanceGrid& distances);

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_HPP
