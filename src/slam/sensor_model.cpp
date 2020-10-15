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
		float rayscore = .001;
        Point<float> rayE;
        rayE.x = ray.range * std::cos(ray.theta) + ray.origin.x;
        rayE.y = ray.range * std::sin(ray.theta)+ ray.origin.y;
        Point<float> rayEnd = global_position_to_grid_position(rayE, map);
		Point<int> rayEndCell = global_position_to_grid_position(rayE, map);
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayStartCell = global_position_to_grid_cell(ray.origin, map);
        rayEndCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayEndCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

		float lods = map.logOdds(rayEndCell.x, rayEndCell.y);

		if(lods > 60 && ray.range < 5.0f){
			rayscore += 150;
		}
		if(ray.range >= 5.0f && lods < 10){
			rayscore += 150;
		}

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
                    if(lods < 10){
						rayscore += .001;
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
		score += rayscore;
	}
    printf("score %f\n", score);
    return score;
}
