#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <common/grid_utils.hpp>
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

void ParticleFilter::initializeFilterAtPoseMap(const pose_xyt_t& pose, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter ////////////////
    posteriorPose_ = pose;
    double sw = 1.0/kNumParticles_;
    std::random_device rd;
    std::mt19937 gen(rd());

    int w = map.widthInCells();
    int h = map.heightInCells();

    for(int i = 0; i<w; i++){
        for(int j = 0; j<h; j++){
            if(map.logOdds(i, j) < 0){
                Point<int> a;
                a.x = i;
                a.y = j;
                emptyCells.push_back(a);
            }
        }
    }

    std::uniform_int_distribution<int> dist(0, emptyCells.size());

    for(auto& p : posterior_){
        Point<double> empty = grid_position_to_global_position(emptyCells[dist(gen)], map);
        p.pose.x = empty.x;
        p.pose.y = empty.y;
        p.pose.theta = pose.theta;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sw;
    }

    std::cout << "Inited particles\n";

}

void ParticleFilter::addNoise(const OccupancyGrid& map)
{
    std::vector<particle_t> noise;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, emptyCells.size());

    for(int i = 0; i<posterior_.size(); i++){
        if(i%8 == 0){
            Point<double> empty = grid_position_to_global_position(emptyCells[dist(gen)], map);
            posterior_[i].pose.x = empty.x;
            posterior_[i].pose.y = empty.y;
        }
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

pose_xyt_t ParticleFilter::updateFilterGuess(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{

    auto prior = resamplePosteriorDistribution();
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    posteriorPose_ = estimatePosteriorPose(posterior_);
    
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
    float avgvx = 0;
    float avgvy = 0;

    for(unsigned int i = 0; i<posterior.size(); i++){
        avgx += posterior[i].pose.x;
        avgy += posterior[i].pose.y;
        avgvx += cos(posterior[i].pose.theta);
        avgvy += sin(posterior[i].pose.theta);
    }

    avgx = avgx/posterior.size();
    avgy = avgy/posterior.size();

    float avg_range = 0;

    for(unsigned int i = 0; i<posterior.size(); i++){
        float dx = posterior[i].pose.x-avgx;
        float dy = posterior[i].pose.y-avgy;
        avg_range += sqrt(dx*dx + dy*dy);
    }

    avg_range = avg_range/posterior.size();

    float pavgx = 0;
    float pavgy = 0;
    int n = 0;
    int j = 0;

    for(unsigned int i = 0; i<posterior.size(); i++){
        float dx = posterior[i].pose.x-avgx;
        float dy = posterior[i].pose.y-avgy;
        if(sqrt(dx*dx + dy*dy) <= avg_range){
            pavgx += posterior[i].pose.x;
            pavgy += posterior[i].pose.y;
            n++;
        }
        if(sqrt(dx*dx + dy*dy) <= .1){
            j++;
        }
    }

    pavgx = pavgx/n;
    pavgy = pavgy/n;

    stability = ((float) j)/kNumParticles_;
    printf("st: %f %f\n", stability, avg_range);

    float avgt = atan2(avgvy,avgvx);

    pose_xyt_t pose;
    pose.x = pavgx;
    pose.y = pavgy;
    pose.theta = avgt;

    return pose;
}
