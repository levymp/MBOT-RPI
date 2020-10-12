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
    // Default to values that are useful for a quick test
    int num_obstacles = 2; // number of obstacles
    float obstacle_spacing = 0.5f; // Distance between obstances
    float amplitude = 0.2; // max distance (along y axis) to travel (peak of sine wave)

    std::cout << "here" <<std::endl;

    if(argc > 1)
    {
        num_obstacles = std::atoi(argv[1]);
    }
    if(argc > 2)
    {
        obstacle_spacing = std::atof(argv[2]);
    }
    if(argc > 3)
    {
        amplitude = std::atof(argv[3]);
    }
    
    std::cout << "Commanding robot to drive around " << num_obstacles << " " << obstacle_spacing << "m spaced obstacles " << std::endl;
    
    float period = obstacle_spacing * 2; // Period of sine wave - 2 obstacles per period
    int num_waypoints = 10 * num_obstacles; // number of discretized points to use in the path

    float path_length = period*(num_obstacles/2.0); // Length (along x) of the path to use
    float point_spacing = path_length/num_waypoints; // Distance between waypoints, currently this is used to evenly space out points along X
    robot_path_t path;
    
    path.path.resize(num_waypoints); // create a path of the chosen size

    for(int idx = 1; idx <= num_waypoints; idx++)
    {
        // Calculate next pose
        pose_xyt_t nextPose;

        nextPose.x = idx*point_spacing;
        nextPose.y = amplitude*sin(M_PI * idx*point_spacing);
        nextPose.theta = std::atan(amplitude*M_PI*cos(M_PI*idx*point_spacing)); // get desired pose from slope: atan(y/x)

        path.path[idx-1] = nextPose;

    }
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
