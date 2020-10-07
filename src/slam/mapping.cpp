#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    // update a map scan, pose, and map (only going to modify the map to update current setup)
    // start and end of scan can be in a different place
    // origin for each ray within the lidar scan may be different
    // since we know each pose we 
    
    // check if we're initialized
    if(!initialized_){
        previousPose_ = pose; 
    }

    // check for initialization
    MovingLaserScan movingScan(scan, previousPose_, pose)

    // go through each ray in the laser scan and score the end point 
    // score end point for each ray (increase log odds @ map locations)
    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
    }

    // go through all rays
    // decrease log odds of cells laser was able to pase through
    for(auto& ray : movingScan){
        scoreRay(ray, map);
    }

    // init -> true
    initialized_ = true;
    // set previous pose
    previousPose_ = pose;

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    // figureout where ray starts and ends and then we need to adjust the log odds of that point

    if(ray.range < kMaxLaserDistance_)
    {
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int> ((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x); 
        rayCell.x = static_cast<int> ((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y); 

        if(map.isCellInGrid(rayCell.x, rayCell.y))
        {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    // TODO
}

void Mapping:: increaseCellOdds(int x, int y, OccupancyGrid& map)
{
    if(initialized_ && map(x,y) < 127)
    {
        map(x, y) += kHitOdds_;
    }
    else if(initialized_ && map(x,y) >= 127)
    {
        map(x, y) = 127;
    }    
}

void Mapping:: decreaseCellOdds(int x, int y, OccupancyGrid& map)
{
    if(initialized_ && map(x,y) > -127)
    {
        map(x, y) -= kMissOdds_;
    }
    else if(initialized_ && map(x,y) <= -127)
    {
        map(x, y) = -127;
    }
}
