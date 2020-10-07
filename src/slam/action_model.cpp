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
    last_pos = cur_pos;
	cur_pos = odometry;

	float delt_x = odometry.x - last_pos.x;
	float delt_y = odometry.y - last_pos.y;

	u_pos[0] = sqrt(delt_x*delt_x + delt_y*delt_y);
	u_pos[1] = angle_diff(atan2(delt_y,delt_x), last_pos.theta);
	u_pos[2] = angle_diff(odometry.theta, u_pos[1]);

	fwd_dist = u_pos[0]*fwd_e;
	a_dist = u_pos[1]*turn_e;
    turn_dist = u_pos[2]*turn_e;

    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

	std::random_device rd;
	std::mt19937 gen(rd());

	std::normal_distribution<float> turn1(0, a_dist);
	std::normal_distribution<float> turn2(0, turn_dist);
	std::normal_distribution<float> fwd(0, fwd_dist);

	float samp_alpha = u_pos[1] + turn1(gen);
	float samp_dalpha = u_pos[2] + turn2(gen);
	float samp_fwd = u_pos[0] + fwd(gen);

	particle_t new_particle;
	new_particle.parent_pose = sample.pose;
	pose_xyt_t npose;
	new_particle.pose = npose;

	new_particle.pose.x = sample.pose.x + samp_fwd*cos(sample.pose.theta + samp_alpha);
	new_particle.pose.y = sample.pose.y + samp_fwd*sin(sample.pose.theta + samp_alpha);
	new_particle.pose.theta = sample.pose.theta + samp_alpha + samp_dalpha;

    return new_particle;
}
