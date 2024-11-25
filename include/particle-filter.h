/**
 * @file particle-filter.h
 * @brief defines the particle filter class, including re-sampling methods
 */

#include "core-structs.h"
#include "math-util.h"
#include "EKF.h"
#include <unordered_map>
#include <queue>
#include <vector>

enum class PF_RET{ SUCCESS = 0, EMPTY_ROBOT_MANAGER = -1, MATRIX_INVERSION_ERROR = -2, UPDATE_ERROR = -3 };
constexpr unsigned int DEFAULT_NUM_PARTICLE = 50;
constexpr float DEFAULT_IMPORTANCE_FACTOR = 0.5;

class FastSLAMParticles {

private:
    /**
     * @brief starting importance factor for determining new lm sightings
     */
    float m_importance_factor;

    /**
     * @brief local copy of the current robot pose, used to ensure timing
     */
    Pose2D m_robot_pose;

    /**
     * @brief current particle data association label
     * @details used to mark current measurement association
     */
    int m_data_label;

    /**
     * @brief importance factor of current observation to landmark
     */
     float m_curr_max_wn;

    /**
     * @brief collection of all landmark EKFs and their
     */
    std::vector<std::pair<std::unique_ptr<LMEKF2D>, int>> m_lmekf_bank;

    /**
     * @brief shared ptr to robot manager instance,
     * needed to instantiate new KFs
     */
     std::shared_ptr<RobotManager2D> m_robot;

    /**
     * @brief get landmark data association label from EKFs given a measurement
     * @details queries EKF for maximum likelihood of correspondence
     *
     * @param[in] curr_obs: current robot observation
     * @return data association index
     */
    int matchLandmark(const struct Observation2D& curr_obs);

    /**
     * @brief update specific landmark belief given new measurement
     *
     * @param[in] curr_obs: current robot observation
     */
    PF_RET updateLMBelief(const struct Observation2D& curr_obs);


#ifdef LM_CLEANUP
    /**
     * @brief clean up dubious features by keeping track of sightings
     * @details keep tracks of landmark sightings using the sightings count
     */
     void cleanUpSightings();
#endif

public:

    FastSLAMParticles() = delete;

    /**
     * @brief class constructor; must provide importance factor, robot starting pose
     * and robot manager
     */
    explicit FastSLAMParticles(float p_0, const struct Pose2D& starting_pose,
                               std::shared_ptr<RobotManager2D> rob_mgr):
    m_importance_factor(p_0), m_robot_pose(starting_pose), m_robot(rob_mgr){
        m_data_label = -1;
        m_curr_max_wn = 0.0f;
    }

    /**
     * @brief copy constructor, used to duplicate particles during the sampling process
     *
     * @param[in] part: particle to copy from
     */
    FastSLAMParticles(const FastSLAMParticles& part);

    /**
     * @brief check for the total number of tracked landmakrs
     * @return current number of tracked landmarks */
    int getNumLandMark() const { return m_lmekf_bank.size(); };

    /**
     * @brief update local copy of current robot pose
     * @param[in] new_pose new pose based on odometry
     */
    PF_RET updatePose(const struct Pose2D& new_pose);

    /**
     * @brief class template method, runs landmark data association and belief update
     *
     * @param[in] new_obs: new robot landmark observation, unclassified
     * @param[in] new_pose: new robot pose estimate
     * @return maximum importance factor for resampling
     */
    const float updateParticle(const struct Observation2D& new_obs);

     /**
     * @brief finds the coordinates of all the landmarks assosciated with a particle
     * @return queue of all the landmark coordinates 
     */
    const std::vector<struct Point2D> getLandmarkCoordinates() const;

    /**
     * @brief get current max importance factor as particle weight
     * @return const reference to current max importance factor
     */
    const float& getParticleWeight() const { return m_curr_max_wn; }
};

class FastSLAMPF {
private:

    /**
     * @brief key-value pairs of index and particles
     */
    std::unordered_map<int, std::unique_ptr<FastSLAMParticles>> m_particle_set;

    /**
     * @brief importance factors associated with particles
     */
    std::vector<float> m_particle_weights;

    /**
     * @brief point to robot manager
     */
    std::shared_ptr<RobotManager2D> m_robot;

    /**
     * @brief number of particles in the filter
     */
    unsigned int m_num_particles;

    /**
     * @brief sample robot pose; this function is probabilistic
     * @details credit: https://stackoverflow.com/questions/6142576
     * /sample-from-multivariate-normal-gaussian-distribution-in-c
     *
     * @param[in] a_pose_mean: collected mean pose from sensor
     * @return a 2D pose sampled from the robot's motion distribution
     */
    struct Pose2D samplePose(const struct Pose2D& a_pose_mean);

    /**
     * @brief resample particles with replacement based on the weights
     */
    void reSampleParticles();

public:

    FastSLAMPF() = delete;

    /**
     * @brief full constructor, need robot manager, number of particles, starting
     * pose, and new landmark importance factor
     *
     * @param[in] rob_ptr: shared pointer to a 2D robot manager
     * @param[in] num_particles: number of particles in the filter
     * @param[in] starting_pose: initial robot pose
     * @param[in] lm_importance_factor: importance factor for landmark MLE
     */
     FastSLAMPF(std::shared_ptr<RobotManager2D> rob_ptr,
                unsigned int num_particles,
                const struct Pose2D& starting_pose,
                const float& lm_importance_factor);

    /**
     * @brief simplified constructor, specifiy robot start around (0,0,0)
     * and use 50 particles
     * @details calls the class delegate constructor specified above
     *
     * @param[in] rob_ptr: shared pointer to a 2D robot manager
     */
     FastSLAMPF(std::shared_ptr<RobotManager2D> rob_ptr);


    /**
     * @brief update and resample particles given new pose mean and lm observation
     *
     * @param[in] a_robot_pose_mean: robot mean pose, sampled directly from sensors
     * @param[in] a_sighting_queue: a queue to all landmark sightings
     */
    void updateFilter(const struct Pose2D& a_robot_pose_mean,
                     std::queue<struct Observation2D>& a_sighting_queue);

    /**
     * @brief extract filter estimate on robot pose and landmark position
     *
     * @return a copied instance of the particle that reflect the SLAM estimations
     * the returned pointer is not the internal state, but rather a copy of the internal state
     */
     std::unique_ptr<FastSLAMParticles> returnEst();

    /**
     * @brief draw one particle (with replacement) based on the random generated sample
     * @details: this method carries out weighted random sampling with a cdf vector; the
     * method needs to be used with a random number generator (see MathUtils::sampleUniform)
     *
     * @param[in] cdf_vec: cdf vector containing cumulative sum of all weights
     * @param[in] sample: randomly-generated number in range of the cdf table
     */
    int drawWithReplacement(const std::vector<float>& cdf_vec, float sample)const;

     /**
     * @brief samples one particle, based on weights, and estimates the landmarks of each EKF
     * assosciated with the particle
     * @return vector of 2DPoints, corresponding to the landmarks associated with that particle
     */
    const std::vector<struct Point2D> sampleLandmarks() const;
};
