#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>


/*
 * Send a sine-wave type path to go around evenly spaced obstacles
 * to the motion controller of choice
 * 
 * To set different values when calling function via command line
 *     ./drive_wave <num_obstacles> <obstacle_spacing> <amplitude>
 */
int main(int argc, char** argv)
{
	robot_path_t path;
    path.path.resize(1);

	pose_xyt_t nextPose;
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);
    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}