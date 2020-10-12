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
		int rayEndCellx = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + ray.origin.x* map.cellsPerMeter());
        int rayEndCelly = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + ray.origin.y* map.cellsPerMeter());
		float lods = map.logOdds(rayEndCellx, rayEndCelly);

		if(lods > 60 && ray.range < 5.0f){
			rayscore += 5;
		}
		if(ray.range >= 5.0f && lods < 10){
			rayscore += 5;
		}

		Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayStartCell = global_position_to_grid_cell(ray.origin, map);

        rayEndCellx = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayEndCelly = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayEndCellx, rayEndCelly)){
            int dx = std::abs(rayEndCellx - rayStartCell.x);
            int dy = std::abs(rayEndCelly - rayStartCell.y);
            int sx = (rayStartCell.x < rayEndCellx) ? 1 : -1;
            int sy = (rayStartCell.y < rayEndCelly) ? 1 : -1;
            int err = dx - dy;
            int x = rayStartCell.x;
            int y = rayStartCell.y;

            while(x != rayEndCellx || y != rayEndCelly){
                if(map.isCellInGrid(x, y)) {
                    if(lods < 10){
						rayscore += 4;
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

    return score;
}
