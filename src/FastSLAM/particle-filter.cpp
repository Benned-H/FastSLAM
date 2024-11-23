/**
 * @file particle-filter.cpp
 * @brief implements the particle filter class
 */

#include "particle-filter.h"
#include "glog/logging.h"

FastSLAMPF::FastSLAMPF(std::shared_ptr<RobotManager2D> rob_ptr,
                       unsigned int num_particles,
                       const struct Pose2D& starting_pose,
                       const float& lm_importance_factor):
    m_robot(rob_ptr),
    m_num_particles(num_particles){

    for (int i = 0; i < m_num_particles; i++) {
        std::unique_ptr<FastSLAMParticles> new_particle = std::make_unique<FastSLAMParticles>
            (lm_importance_factor, starting_pose, m_robot);
        auto particle_pair = std::make_pair(i, std::move(new_particle));
        m_particle_set.insert(std::move(particle_pair));

        m_particle_weights.push_back(1.0f / static_cast<float>(m_num_particles));
    }
}


FastSLAMPF::FastSLAMPF(std::shared_ptr<RobotManager2D> rob_ptr):
    FastSLAMPF(rob_ptr, DEFAULT_NUM_PARTICLE,
               {.x = 0, .y = 0, .theta_rad = 0}, DEFAULT_IMPORTANCE_FACTOR) {
}

struct Pose2D FastSLAMPF::samplePose(const struct Pose2D& a_pose_mean) {
    Eigen::Matrix3f l_cholesky;
    Eigen::LLT<Eigen::Matrix3f> cholSolver(m_robot->getProcessNoise());
    LOG(INFO) << "Robot process noise covariance matrix: \n" << m_robot->getProcessNoise();
    LOG(INFO) << "Using choleksy solver? " << (cholSolver.info() == Eigen::Success);
    if (cholSolver.info()==Eigen::Success) {
        // Use cholesky solver
        l_cholesky = cholSolver.matrixL();
    } else {
        // Use eigen solver
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(m_robot->getProcessNoise());
        l_cholesky = eigenSolver.eigenvectors()
            * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }
    Eigen::Vector3f z = Eigen::Vector3f::Zero();
    for (auto& it: z){
        it = MathUtil::sampleNormal(0.0f, 1.0f);
    }
    LOG(INFO) << "Sampled z is : \n" << z;
    struct Pose2D ret = a_pose_mean;
    ret += l_cholesky * z;
    LOG(INFO) << "Return pose: x " << ret.x  << "; y " << ret.y << "; theta " << ret.theta_rad;
    return ret;
}

int FastSLAMPF::drawWithReplacement(const std::vector<float>& cdf_vec, float sample) const{
    int start = 0;
    int end = cdf_vec.size()-1;
    if (sample < 0 || sample > cdf_vec[end]) return -1;

    // binary search, if the sample falls between the [prev, next) interval,
    // then consider the sample drawn from that interval
    int middle;
    while (start != end) {
        middle = (start + end) / 2;
        if (sample >= cdf_vec[middle]) {
            start = middle+1;
        } else {
            end = middle;
        }
    }
    return middle;
}

void FastSLAMPF::reSampleParticles(){
    std::vector<float> cdf_table;
    float total_weight = MathUtil::genCDF(m_particle_weights, cdf_table);
    float sampled_weight;
    std::unordered_map<int, std::unique_ptr<FastSLAMParticles>> aux_set;

    for (const auto& it: m_particle_set){
        sampled_weight = MathUtil::sampleUniform(0.0, total_weight);
        LOG(INFO) << "Sampled weight: " << sampled_weight;
        int sampled_idx = drawWithReplacement(cdf_table, sampled_weight);
        LOG(INFO) << "Sampled idx: " << sampled_idx;
        // leave original particle if sampling goes wrong
        sampled_idx = sampled_idx >= 0 ? sampled_idx : it.first;
        std::unique_ptr<FastSLAMParticles> particle_drew =
            std::make_unique<FastSLAMParticles>(*m_particle_set[sampled_idx]);
        aux_set.insert({it.first, std::move(particle_drew)});
    }

    m_particle_set = std::move(aux_set);

}

void FastSLAMPF::updateFilter(const struct Pose2D &a_robot_pose_mean,
                         std::queue<struct Observation2D> &a_sighting_queue) {
    while (!a_sighting_queue.empty()){
        int idx = 0;
        for (auto& it: m_particle_set){
            LOG(INFO) << "Updating particle #" << it.first;
            auto rob_pose_sampled = samplePose(a_robot_pose_mean);
            m_particle_weights[idx] += it.second->updateParticle(
                a_sighting_queue.front(), rob_pose_sampled);
            LOG(INFO) << "resulting particle weight: " << m_particle_weights[idx];
            idx++;
        }
        a_sighting_queue.pop();
    }
    reSampleParticles();
}

const std::vector<struct Point2D> FastSLAMPF::sampleLandmarks() const {
    std::vector<float> cdf_table;
    float total_weight = MathUtil::genCDF(m_particle_weights, cdf_table);
    float sampled_weight = MathUtil::sampleUniform(0.0, total_weight);
    int sampled_idx = drawWithReplacement(cdf_table, sampled_weight); 
    return (m_particle_set.at(sampled_idx))->getLandmarkCoordinates();  
}
