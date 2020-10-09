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
		float rayscore = 0;
		float lods = map.logOdds(ray.origin.x + ray.range*cos(ray.theta), ray.origin.y + ray.range*sin(ray.theta));
		if(lods > 60){
			rayscore += 5;
		}
		if(lods > 20){
			rayscore += 2;
		}
		score += rayscore;
	}

    return score;
}
