#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{

    std::cout << "Commanding robot to drive 1m, turn, and return back.\n";
    
    robot_path_t path;
    path.path.resize(5);
    
    pose_xyt_t nextPose;

    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[0] = nextPose;
    
    nextPose.x = 0.5f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[1] = nextPose;

    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI;
    path.path[2] = nextPose;

    nextPose.x = 0.5f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI;
    path.path[3] = nextPose;

    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[4] = nextPose;
    
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
