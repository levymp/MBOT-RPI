#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(0)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    
    if(!initialized_){
        previousPose_ = pose;
        initialized_ = 1;
    }

    MovingLaserScan movingScan {scan, previousPose_, pose};

    for(auto &ray : movingScan){
        scoreEndpoint(ray, map);
    }

    for(auto &ray : movingScan){
        scoreRay(ray, map);
    }

    previousPose_ = pose;

}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid &map){
    
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map){
    
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayStartCell = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayEndCell;

        Point<float> rayEnd;
        rayEnd.x = (ray.range * std::cos(ray.theta) + ray.origin.x);
        rayEnd.y = (ray.range * std::cos(ray.theta) + ray.origin.y);
        rayEndCell = global_position_to_grid_cell(rayEnd, map);
        //rayEndCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        //rayEndCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayEndCell.x, rayEndCell.y)){
            int dx = std::abs(rayEndCell.x - rayStartCell.x);
            int dy = std::abs(rayEndCell.y - rayStartCell.y);
            int sx = (rayStartCell.x < rayEndCell.x) ? 1 : -1;
            int sy = (rayStartCell.y < rayEndCell.y) ? 1 : -1;
            int err = dx - dy;
            int x = rayStartCell.x;
            int y = rayStartCell.y;

            while(x != rayEndCell.x || y != rayEndCell.y){
                if(map.isCellInGrid(x, y)) {
                    decreaseCellOdds(x, y, map);
                }
                int e2 = 2 * err;
                if(e2 >= -dy){
                    err -= dy;
                    x += sx;
                }
                if(e2 <= dx){
                    err += dx;
                    y += sy;
                }
            }
        }
    }
}

void Mapping::increaseCellOdds(const int x, const int y, OccupancyGrid &map){

    if(!initialized_){
        // do nothing;
    }
    else if(map(x,y) + kHitOdds_ < 127){
        map(x,y) += kHitOdds_;  
    }
    else{
        map(x,y) = 127;
    }

}

void Mapping::decreaseCellOdds(const int x, const int y, OccupancyGrid &map){
    
    if(!initialized_){
        // do nothing;
    }
    else if(map(x,y) - kMissOdds_ > -128){
        map(x,y) -= kMissOdds_;  
    }
    else{
        map(x,y) = -128;
    }

}
