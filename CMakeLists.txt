cmake_minimum_required(VERSION 3.16)
project(humanoid_robot_stack LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

option(HUMANOID_ENABLE_PINOCCHIO "Enable the real Pinocchio/URDF kinematics backend when a URDF model is provided" OFF)
set(HUMANOID_PINOCCHIO_URDF_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/humanoid_system/models/unitree_g1_12dof/g1_12dof.urdf"
  CACHE STRING "Default URDF path used by the optional Pinocchio backend")
set(HUMANOID_PINOCCHIO_LEFT_FOOT_FRAME
  "left_sole_link"
  CACHE STRING "Default left foot frame name used by the optional Pinocchio backend")
set(HUMANOID_PINOCCHIO_RIGHT_FOOT_FRAME
  "right_sole_link"
  CACHE STRING "Default right foot frame name used by the optional Pinocchio backend")
set(HUMANOID_PINOCCHIO_JOINT_ORDER
  "left_hip_pitch_joint,left_hip_roll_joint,left_hip_yaw_joint,left_knee_joint,left_ankle_pitch_joint,left_ankle_roll_joint,right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,right_knee_joint,right_ankle_pitch_joint,right_ankle_roll_joint"
  CACHE STRING "Comma-separated joint names mapping state.q_j order to the optional Pinocchio backend")
option(HUMANOID_PINOCCHIO_FLOATING_BASE
  "Build the optional Pinocchio model with a free-flyer root joint" OFF)
option(HUMANOID_PINOCCHIO_STRICT_MODEL_CHECK
  "Treat invalid Pinocchio URDF/frame/joint_order configuration as a real-model validation error" ON)
set(HUMANOID_IMU_TO_BASE_RPY
  "0,0,0"
  CACHE STRING "Fixed rotation from IMU frame to base/body frame, as roll,pitch,yaw in radians")
set(HUMANOID_IMU_IN_BASE_TRANSLATION
  "0,0,0"
  CACHE STRING "Fixed IMU position in base/body frame, as x,y,z in meters")
if (HUMANOID_ENABLE_PINOCCHIO)
  # Pinocchio 已经可以通过 ROS Noetic 包安装到 /opt/ros/noetic。
  # 默认 URDF 已切换为宇树官方开源 G1 12DoF 模型，比教学 6DoF 模型更接近真实机器人；
  # 如果接入自己的机器人，可通过 CMake cache 或环境变量替换 URDF、足端 frame 和关节顺序。
  find_package(pinocchio REQUIRED CONFIG PATHS /opt/ros/noetic)
endif()

add_executable(humanoid_system_demo
  humanoid_system/main.cpp
  humanoid_system/estimation/eskf_whole_body.cpp
  humanoid_system/estimation/imu_propagation.cpp
  humanoid_system/estimation/kinematics_pinocchio.cpp
  humanoid_system/estimation/contact_estimator.cpp
  humanoid_system/robot_model/robot_model_config.cpp
  humanoid_system/world_model/semantic_map.cpp
  humanoid_system/world_model/object_tracker.cpp
  humanoid_system/world_model/occupancy_grid.cpp
  humanoid_system/planning/cost_map.cpp
  humanoid_system/planning/constraint_generator.cpp
  humanoid_system/simulation/humanoid_sim.cpp
  humanoid_system/simulation/sensor_sim.cpp
  humanoid_system/simulation/scenario_generator.cpp
  humanoid_system/system/data_bus.cpp
  humanoid_system/system/sensor_buffer.cpp
  humanoid_system/system/state_history_buffer.cpp
  humanoid_system/system/real_time_loop.cpp
)

# Pinocchio版本为 ros-noetic-pinocchio 3.3.0， Eigen 版本为 3.3.7。
target_include_directories(humanoid_system_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(humanoid_system_demo PRIVATE Eigen3::Eigen)
target_compile_definitions(humanoid_system_demo PRIVATE HUMANOID_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(humanoid_system_demo PRIVATE
  HUMANOID_DEFAULT_IMU_TO_BASE_RPY="${HUMANOID_IMU_TO_BASE_RPY}"
  HUMANOID_DEFAULT_IMU_IN_BASE_TRANSLATION="${HUMANOID_IMU_IN_BASE_TRANSLATION}"
  HUMANOID_PINOCCHIO_DEFAULT_URDF="${HUMANOID_PINOCCHIO_URDF_PATH}"
  HUMANOID_PINOCCHIO_DEFAULT_LEFT_FOOT="${HUMANOID_PINOCCHIO_LEFT_FOOT_FRAME}"
  HUMANOID_PINOCCHIO_DEFAULT_RIGHT_FOOT="${HUMANOID_PINOCCHIO_RIGHT_FOOT_FRAME}"
  HUMANOID_PINOCCHIO_DEFAULT_JOINT_ORDER="${HUMANOID_PINOCCHIO_JOINT_ORDER}"
)
if (HUMANOID_PINOCCHIO_FLOATING_BASE)
  target_compile_definitions(humanoid_system_demo PRIVATE HUMANOID_PINOCCHIO_DEFAULT_FLOATING_BASE=1)
else()
  target_compile_definitions(humanoid_system_demo PRIVATE HUMANOID_PINOCCHIO_DEFAULT_FLOATING_BASE=0)
endif()
if (HUMANOID_PINOCCHIO_STRICT_MODEL_CHECK)
  target_compile_definitions(humanoid_system_demo PRIVATE HUMANOID_PINOCCHIO_DEFAULT_STRICT_MODEL_CHECK=1)
else()
  target_compile_definitions(humanoid_system_demo PRIVATE HUMANOID_PINOCCHIO_DEFAULT_STRICT_MODEL_CHECK=0)
endif()

if (HUMANOID_ENABLE_PINOCCHIO)
  target_link_options(humanoid_system_demo PRIVATE "-Wl,--no-as-needed")
  target_link_libraries(humanoid_system_demo PRIVATE pinocchio::pinocchio)
  target_link_libraries(humanoid_system_demo PRIVATE /opt/ros/noetic/lib/x86_64-linux-gnu/libhpp-fcl.so)
  target_compile_definitions(humanoid_system_demo PRIVATE HUMANOID_ENABLE_PINOCCHIO=1)
  set_target_properties(humanoid_system_demo PROPERTIES
    BUILD_RPATH "/opt/ros/noetic/lib;/opt/ros/noetic/lib/x86_64-linux-gnu"
  )
  target_link_options(humanoid_system_demo PRIVATE "-Wl,--disable-new-dtags")
endif()

if (MSVC)
  target_compile_options(humanoid_system_demo PRIVATE /W4)
else()
  target_compile_options(humanoid_system_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()
