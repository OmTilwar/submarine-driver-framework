cmake_minimum_required(VERSION 3.16)
project(submarine_drivers VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
    add_compile_options(/W4)
    add_definitions(-D_USE_MATH_DEFINES)
endif()

# --- Option: build with or without ROS2 ---
option(BUILD_ROS2_NODES "Build ROS2 node wrappers (requires ament_cmake)" OFF)

# --- Core Library (pure C++, no ROS2 dependency) ---
set(CORE_SOURCES
    src/core/noise_model.cpp
    src/sensors/imu_driver.cpp
    src/sensors/dvl_driver.cpp
    src/sensors/depth_sensor_driver.cpp
    src/actuators/thruster_driver.cpp
    src/estimation/state_estimator_6dof.cpp
)

add_library(submarine_drivers_core ${CORE_SOURCES})
target_include_directories(submarine_drivers_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(submarine_drivers_core PUBLIC cxx_std_17)

# --- Tests (GTest) ---
option(BUILD_TESTING "Build unit tests" ON)

if(BUILD_TESTING)
    # Try to find GTest
    find_package(GTest QUIET)

    if(NOT GTest_FOUND)
        # Fetch GTest if not installed
        include(FetchContent)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        v1.14.0
        )
        # Prevent overriding the parent project's compiler/linker settings (Windows)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()

    enable_testing()

    set(TEST_SOURCES
        test/test_noise_model.cpp
        test/test_thruster_driver.cpp
        test/test_imu_driver.cpp
        test/test_dvl_driver.cpp
        test/test_depth_sensor.cpp
        test/test_state_estimator.cpp
    )

    add_executable(submarine_drivers_tests ${TEST_SOURCES})
    target_link_libraries(submarine_drivers_tests
        PRIVATE
            submarine_drivers_core
            GTest::gtest
            GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(submarine_drivers_tests)
endif()

# --- ROS2 Nodes (conditional) ---
if(BUILD_ROS2_NODES)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)

    # Sensor driver node
    add_executable(sensor_driver_node src/nodes/sensor_driver_node.cpp)
    target_link_libraries(sensor_driver_node submarine_drivers_core)
    ament_target_dependencies(sensor_driver_node
        rclcpp std_msgs geometry_msgs sensor_msgs)

    # State estimator node
    add_executable(state_estimator_node src/nodes/state_estimator_node.cpp)
    target_link_libraries(state_estimator_node submarine_drivers_core)
    ament_target_dependencies(state_estimator_node
        rclcpp std_msgs geometry_msgs sensor_msgs)

    # Trajectory simulation node
    add_executable(trajectory_sim_node src/nodes/trajectory_sim_node.cpp)
    target_link_libraries(trajectory_sim_node submarine_drivers_core)
    ament_target_dependencies(trajectory_sim_node
        rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros)

    install(TARGETS sensor_driver_node state_estimator_node trajectory_sim_node
        DESTINATION lib/${PROJECT_NAME})

    install(DIRECTORY launch config urdf
        DESTINATION share/${PROJECT_NAME})

    ament_package()
endif()

# --- Install (standalone mode) ---
if(NOT BUILD_ROS2_NODES)
    install(TARGETS submarine_drivers_core
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
    install(DIRECTORY include/ DESTINATION include)
endif()
