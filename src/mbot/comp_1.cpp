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
    int numTimes = 4;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    float square_size = 0.75f;

    std::cout << "Commanding robot to drive around " << square_size << "m square " << numTimes << " times.\n";
    
    robot_path_t path;
    path.path.resize(numTimes * 8 + 1);
    
    pose_xyt_t nextPose;

    nextPose.x = square_size*.5;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 1] = nextPose;
    }

    nextPose.x = square_size;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 2] = nextPose;
    }

    nextPose.x = square_size;
    nextPose.y = square_size*.5;
    nextPose.theta = M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 3] = nextPose;
    }
    
    nextPose.x = square_size;
    nextPose.y = square_size;
    nextPose.theta = M_PI;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 4] = nextPose;
    }

    nextPose.x = square_size*.5;
    nextPose.y = square_size;
    nextPose.theta = M_PI;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 5] = nextPose;
    }

    nextPose.x = 0.0f;
    nextPose.y = square_size;
    nextPose.theta = -M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 6] = nextPose;
    }

    nextPose.x = 0.0f;
    nextPose.y = square_size*.5;
    nextPose.theta = -M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 7] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[8*n + 8] = nextPose;
    }
    
    // Return to original heading after completing all circuits
    //    nextPose.theta = 0.0f;
    //    path.path.push_back(nextPose);
    
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
