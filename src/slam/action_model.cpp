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

    std::random_device rd;
	gen = std::mt19937(rd());
	inited = false;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!inited){
    	last_pos = odometry;
    	inited = true;
    }

	float delt_x = odometry.x - last_pos.x;
	float delt_y = odometry.y - last_pos.y;
	float delt_t = angle_diff(odometry.theta,last_pos.theta);
	float direction = 1;

	u_pos[0] = sqrt(delt_x*delt_x + delt_y*delt_y);
	u_pos[1] = angle_diff(atan2(delt_y,delt_x), last_pos.theta);
	if(fabs(u_pos[1]) > M_PI_2){
		u_pos[1] = angle_diff(M_PI, u_pos[1]);
		direction = -1;
	}
	u_pos[2] = angle_diff(delt_t, u_pos[1]);

	if(delt_x == 0 && delt_y == 0 && delt_t == 0){
		moved = false;
		return false;
	}

    last_pos = odometry;
    utime = odometry.utime;

	fwd_dist = sqrt(u_pos[0]*fwd_e);
	a_dist = sqrt(u_pos[1]*turn_e);
    turn_dist = sqrt(u_pos[2]*turn_e);

    moved = true;
    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_particle;
	if(moved){
		std::normal_distribution<float> turn1(0, a_dist);
		std::normal_distribution<float> turn2(0, turn_dist);
		std::normal_distribution<float> fwd(0, fwd_dist);

		float samp_alpha = u_pos[1] + turn1(gen);
		float samp_dalpha = u_pos[2] + turn2(gen);
		float samp_fwd = u_pos[0] + fwd(gen);

		
		new_particle.pose.utime = utime;
		new_particle.parent_pose = sample.pose;
		new_particle.pose.x = sample.pose.x + samp_fwd*cos(sample.pose.theta + samp_alpha);
		new_particle.pose.y = sample.pose.y + samp_fwd*sin(sample.pose.theta + samp_alpha);
		new_particle.pose.theta = wrap_to_pi(sample.pose.theta + samp_alpha + samp_dalpha);
	} else {

		new_particle.pose.utime = utime;
		new_particle.parent_pose = sample.pose;
		new_particle.pose.x = sample.pose.x;
		new_particle.pose.y = sample.pose.y;
		new_particle.pose.theta = sample.pose.theta;
	}

    return new_particle;
}
