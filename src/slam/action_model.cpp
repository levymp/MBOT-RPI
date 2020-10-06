#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    turn_e = .01;
    fwd_e = .01;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

	float delt_x = odometry->x - lastp.x;
	float delt_y = odometry->y - lastp.y;

	float delt_s = sqrt(delt_x^2 + delt_y^2);
	float alpha = atan2(delt_y/delt_x) - lastp.theta;
	float delt_a = odometry->theta - alpha;

	float[3] u_pos = {delt_s, alpha, delt_a};

	fwd_dist = delt_s*fwd_e;
	a_dist = alpha*turn_e;
    turn_dist = delt_a*turn_e;
    

    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    return sample;
}
