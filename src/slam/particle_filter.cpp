#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <unistd.h>
#include <cassert>
#include <random>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter ////////////////
    posteriorPose_ = pose;
    double sw = 1.0/kNumParticles_;

    for(auto& p : posterior_){
        p.pose.x = pose.x;
        p.pose.y = pose.y;
        p.pose.theta = pose.theta;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sw;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    auto prior = resamplePosteriorDistribution();
    
    if(hasRobotMoved)
    {
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    std::vector<particle_t> prior;

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<double> weights;
    for(auto& p : posterior_){
        weights.push_back(p.weight);
    }

    std::discrete_distribution<int> w_dist(weights.begin(), weights.end());

    for(auto& p : posterior_){
        prior.push_back(posterior_[w_dist(gen)]);
    }
    
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel

    std::vector<particle_t> proposal;

    for(auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution

    std::vector<particle_t> posterior;
    double tot_w = 0;

    for(unsigned int i=0; i<proposal.size(); i++){
        particle_t part = proposal[i];
        part.weight = sensorModel_.likelihood(proposal[i], laser, map);
        tot_w += part.weight;
        posterior.push_back(part);
    }

    for(unsigned int i=0; i<proposal.size(); i++){
        posterior[i].weight = posterior[i].weight/tot_w;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    float avgx = 0;
    float avgy = 0;
    float avgt = 0;

    for(unsigned int i = 0; i<posterior.size(); i++){
        avgx += posterior[i].pose.x;
        avgy += posterior[i].pose.y;
        avgt = avgt + M_PI;
    }

    avgx = avgx/posterior.size();
    avgy = avgy/posterior.size();
    avgt = avgt/posterior.size() - M_PI;

    pose_xyt_t pose;
    pose.x = avgx;
    pose.y = avgy;
    pose.theta = avgt;

    return pose;
}
