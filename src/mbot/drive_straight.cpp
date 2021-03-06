#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{
    std::cout << "here" <<std::endl;
    int numTimes = 1;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    float path_length = 0.5f;

    std::cout << "Commanding robot to drive " << path_length << "m forward and back" << numTimes << " times.\n";
    
    robot_path_t path;
    path.path.resize(numTimes * 2 + 1);
    
    pose_xyt_t nextPose;
    
    nextPose.x = path_length;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[2*n] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[2*n + 1] = nextPose;
    }
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
