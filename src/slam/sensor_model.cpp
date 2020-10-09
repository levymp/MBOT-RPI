#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
	float score = 0;
	MovingLaserScan mvscan(scan, sample.parent_pose, sample.pose);

	for(const auto& ray : mvscan){
		float rayscore = 1;
		int rayCellx = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + ray.origin.x* map.cellsPerMeter());
        int rayCelly = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + ray.origin.y* map.cellsPerMeter());
		float lods = map.logOdds(rayCellx, rayCelly);

		if(lods > 60 && ray.range < 5.0f){
			rayscore += 5;
		}

		if(map.isCellInGrid(rayCellx, rayCelly)){
            int dx = std::abs(rayCellx - ray.origin.x);
            int dy = std::abs(rayCelly - ray.origin.y);
            int sx = (ray.origin.x < rayCellx) ? 1 : -1;
            int sy = (ray.origin.y < rayCelly) ? 1 : -1;
            int err = dx - dy;
            int x = ray.origin.x;
            int y = ray.origin.y;

            while(x != rayCellx || y != rayCelly){
                if(map.isCellInGrid(x, y)) {
                    if(lods < 10){
						rayscore += 1;
					}
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

		if(ray.range >= 5.0f && lods < 10){
			rayscore += 8;
		}
		score += rayscore;
	}

    return score;
}
