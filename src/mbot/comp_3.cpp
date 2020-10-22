#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{
	mbot_motor_command_t cmd;
	lcm::LCM lcmInstance(MULTICAST_URL);

    cmd.trans_v = 0.0f;
    cmd.angular_v = .3f;
    lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);

    sleep(5);

    cmd.trans_v = 0.1f;
    cmd.angular_v = 0.0f;
	lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);

	sleep(2);

	cmd.trans_v = -0.1f;
    cmd.angular_v = 0.0f;
	lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);

	sleep(4);

    cmd.trans_v = 0.0f;
    cmd.angular_v = 0.0f;
	lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);

    return 0;
}