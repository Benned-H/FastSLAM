add_library(FastSLAMLib
   math-util.cpp
   EKF.cpp
   create3-manager.cpp
   particle-filter.cpp
   particles.cpp
)
if(USE_MOCK)
    target_sources(FastSLAMLib PUBLIC mock-manager2d.cpp)
endif()

target_include_directories(FastSLAMLib PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(FastSLAMLib 
                      Eigen3::Eigen
                      glog::glog
                      gflags)

add_executable(fastslam fastslam.cpp)
target_link_libraries(fastslam FastSLAMLib)

if(BUILD_TESTS)
  add_executable(test_MathUtil math-util_test.cpp)
  target_link_libraries(test_MathUtil
                        PRIVATE Catch2::Catch2WithMain
                        FastSLAMLib)
  catch_discover_tests(test_MathUtil)
  target_include_directories(test_MathUtil PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
  )

  add_executable(test_EKF EKF_test.cpp)
  add_executable(test_Particle particle-filter_test.cpp)

  if(USE_MOCK)
    target_compile_definitions(test_EKF PUBLIC USE_MOCK)
    target_compile_definitions(test_Particle PUBLIC USE_MOCK)
    add_executable(test_MockManager2d mock-manager2d_test.cpp)
    target_compile_definitions(test_MockManager2d PUBLIC USE_MOCK)
    target_link_libraries(test_MockManager2d
                        PRIVATE Catch2::Catch2WithMain
                        FastSLAMLib)
    catch_discover_tests(test_MockManager2d)
    target_include_directories(test_MockManager2d PUBLIC
      "${PROJECT_BINARY_DIR}"
      "${PROJECT_SOURCE_DIR}/include"
    )
  else()
    target_compile_definitions(test_EKF PUBLIC USE_SIM)
  endif()

  if (LM_CLEANUP)
    target_compile_definitions(FastSLAMLib PUBLIC LM_CLEANUP)
  endif()

  target_link_libraries(test_EKF
                        PRIVATE Catch2::Catch2WithMain
                        FastSLAMLib)
  catch_discover_tests(test_EKF)
  target_include_directories(test_EKF PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
  )

  target_link_libraries(test_Particle
                        PRIVATE Catch2::Catch2WithMain
                        FastSLAMLib)
  catch_discover_tests(test_Particle)
  target_include_directories(test_Particle PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
  )
endif()
