#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "robot-manager.h"
#include "particle-filter.h"

TEST_CASE( "Default Particle" ){
    // set-up
    struct Pose2D init_pose = { .x = 0, .y =0, .theta_rad = 0 };
    struct VelocityCommand2D init_cmd = { .vx_mps = 1, .wz_radps = 0 };
    struct Point2D init_obs = { .x = 0, .y = 1 };
    Eigen::Matrix2f init_cov;
    init_cov << 1, 0,
        0, 1;

#ifdef USE_MOCK
    std::shared_ptr<RobotManager2D> test_manager =
        std::make_shared<MockManager2D>(init_pose, init_cmd, init_cov, 0, Eigen::Matrix3f::Zero());
    std::unique_ptr<FastSLAMParticles> test_particle =
        std::make_unique<FastSLAMParticles>(0.5, init_pose, test_manager);
#else
    std::unique_ptr<FastSLAMParticles> test_particle =
        std::make_unique<FastSLAMParticles>(0.5, init_pose, nullptr);
#endif //USE_MOCK

    REQUIRE( test_particle->getNumLandMark() == 0 );

    SECTION( "Test copy construction" ){
        std::unique_ptr<FastSLAMParticles> copied_particle =
            std::make_unique<FastSLAMParticles>(*test_particle);
        REQUIRE(copied_particle != nullptr);
        REQUIRE(copied_particle->getNumLandMark() == 0);
    }

    SECTION( "Test update landmark" ){
        float res = test_particle->updateParticle({.range_m = 1, .bearing_rad = 0},
                                      {.x = 0, .y = 1, .theta_rad = 0});
        REQUIRE(res != static_cast<float>(PF_RET::UPDATE_ERROR));
        REQUIRE(test_particle->getNumLandMark() == 1);
    }

}

TEST_CASE( "Test Particle Filter" ){
    // set-up
    struct Pose2D init_pose = { .x = 0, .y =0, .theta_rad = 0 };
    struct VelocityCommand2D init_cmd = { .vx_mps = 1, .wz_radps = 0 };
    struct Point2D init_obs = { .x = 0, .y = 1 };
    Eigen::Matrix2f init_cov;
    init_cov << 1, 0,
        0, 1;

#ifdef USE_MOCK
    std::shared_ptr<RobotManager2D> test_manager =
        std::make_shared<MockManager2D>(init_pose, init_cmd, init_cov, 0, Eigen::Matrix3f::Zero());
    std::unique_ptr<FastSLAMPF> test_pf =
        std::make_unique<FastSLAMPF>(test_manager);
#else
    std::unique_ptr<FastSLAMPF> test_pf = std::make_unique<FastSLAMPF>(nullptr);
#endif //USE_MOCK

    std::vector<float> cdf_table = {0.1, 0.4, 0.8, 1};

    REQUIRE( test_pf->drawWithReplacement(cdf_table, 0.5) == 2);
    REQUIRE( test_pf->drawWithReplacement(cdf_table, 0.09) == 0);
    REQUIRE( test_pf->drawWithReplacement(cdf_table, 0.4) == 2);
    REQUIRE( test_pf->drawWithReplacement(cdf_table, 2) == -1);

}
