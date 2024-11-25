#include "particle-filter.h"
#include "robot-manager.h"
#include <glog/logging.h>
#include <stdio.h>
#include <memory.h>

// constants for create3 pose sensor noise
static const float x_accel_var = 0.0001543f;
static const float y_accel_var = 0.0001636f;
static const float theta_var = 0.0000125f;

// static const struct Observation2D obs = {.range_m = 0.623143,
//                                         .bearing_rad = 0.987187};
constexpr Observation2D obs = {.range_m = 1.0f, .bearing_rad = 0.0f};

int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);

    const struct Pose2D init_pose = {.x = 0, .y = 0, .theta_rad = 0};
    const struct VelocityCommand2D init_cmd {.vx_mps = 0, .wz_radps = 0};
    Eigen::Matrix3f rob_process_noise = Eigen::Matrix3f::Zero();
    rob_process_noise.diagonal() << x_accel_var, y_accel_var, theta_var;
    std::shared_ptr<Create3Manager> m_robot_manager = std::make_shared<Create3Manager>(init_pose, init_cmd, 
        0.5 * Eigen::Matrix2f::Ones(), 3.0f, rob_process_noise);
    std::unique_ptr<FastSLAMPF> m_fastslam_filter = std::make_unique<FastSLAMPF>(
        std::static_pointer_cast<RobotManager2D>(m_robot_manager), 2, init_pose, 0.5);

    std::queue<Observation2D> lidar_landmarks;
    lidar_landmarks.push(obs);
    lidar_landmarks.push(obs);

    m_fastslam_filter->updateFilter(init_pose, lidar_landmarks);


    return 0;
}
