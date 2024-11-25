/**
 * @file EKF.cpp
 * @brief Implements the EKF derived classes
 */

#include "EKF.h"
#include "glog/logging.h"


LMEKF2D::LMEKF2D() {
   m_mu = { .x  = 0 , .y = 0 };
   m_sigma = Eigen::Matrix2f::Zero();
   m_curr_obs = { .range_m = 0, .bearing_rad = 0, .landmarkID = std::nullopt};
   m_robot = nullptr;
   m_meas_cov = Eigen::Matrix2f::Zero();
}

LMEKF2D::LMEKF2D(struct Point2D init_obs, Eigen::Matrix2f init_cov,
                 std::shared_ptr<RobotManager2D> robot_ptr) : m_mu(init_obs),
   m_sigma(init_cov),
   m_robot(robot_ptr) {
   m_curr_obs = { .range_m = 0, .bearing_rad = 0, .landmarkID = std::nullopt};
   m_meas_cov = Eigen::Matrix2f::Zero();
}

LMEKF2D::LMEKF2D(const LMEKF2D& ekf):
    m_mu(ekf.m_mu),m_sigma(ekf.m_sigma),
    m_robot(ekf.m_robot), m_meas_cov(ekf.m_meas_cov), m_curr_obs(ekf.m_curr_obs){
}

Eigen::Matrix2f LMEKF2D::measJacobian() const {
    if (m_robot == nullptr) {
        return Eigen::Matrix2f::Zero();
    }
    return m_robot->measJacobian(m_mu); // make sure this is called before update()
}

void LMEKF2D::calcMeasCov() {
    Eigen::Matrix2f G_n = this->measJacobian();
    LOG(INFO) << "Measurement jacobian: \n" << G_n;
    m_meas_cov = G_n.transpose() * m_sigma * G_n + m_robot->getMeasNoise();
}

Eigen::Matrix2f LMEKF2D::calcKalmanGain() const {
    if (m_robot == nullptr || m_meas_cov.determinant() == 0) {
        return Eigen::Matrix2f::Zero();
    }

    return m_sigma * this->measJacobian() * (m_meas_cov.inverse());
}

KF_RET LMEKF2D::update() {
    if (m_robot == nullptr) {
        return KF_RET::EMPTY_ROBOT_MANAGER;
    }
    this->calcMeasCov();
    if (m_meas_cov.determinant() == 0) {
        return KF_RET::MATRIX_INVERSION_ERROR;
    } else {
        Eigen::Matrix2f G_n = this->measJacobian();
        Eigen::Matrix2f K = this->calcKalmanGain();

        m_mu += K * ( m_curr_obs - m_robot->predictMeas(m_mu) );
        
        m_sigma = ( Eigen::Matrix2f::Identity() - K * G_n.transpose() ) * m_sigma;
    }

    return KF_RET::SUCCESS;
}

float LMEKF2D::calcCPD() {
    if (m_robot == nullptr) {
        LOG(ERROR) << "Empty robot manager";
        return -1.0f;
    }

    LOG(INFO) << "Calculating CPD";
    this->calcMeasCov();
    if (m_meas_cov.determinant() == 0) {
        LOG(WARNING) << "Measurement covariance doesn't have full rank";
        return -1.0f;
    }

    Eigen::Vector2f residue = m_curr_obs - m_robot->predictMeas(m_mu);
    LOG(INFO) << "residue: " << residue;

    float weight = 1 / sqrtf( (2 * M_PI * m_meas_cov).determinant() );
    weight = weight * expf( -0.5 * residue.transpose() * m_meas_cov.inverse() * residue );
    LOG(INFO) << "weight: " << weight;

    return weight;
}

const struct Point2D& LMEKF2D::getLMEst() const {
   return m_mu;
}

void LMEKF2D::updateObservation(const struct Observation2D &new_obs) {
    LOG(INFO) << "Updating observation: range=" << new_obs.range_m
              << ", bearing=" << new_obs.bearing_rad << std::endl;
    m_curr_obs = new_obs;
}
