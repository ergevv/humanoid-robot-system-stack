cmake_minimum_required(VERSION 3.16)
project(humanoid_robot_stack LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(humanoid_system_demo
  humanoid_system/main.cpp
  humanoid_system/estimation/eskf_whole_body.cpp
  humanoid_system/estimation/imu_propagation.cpp
  humanoid_system/estimation/kinematics_pinocchio.cpp
  humanoid_system/estimation/contact_estimator.cpp
  humanoid_system/world_model/semantic_map.cpp
  humanoid_system/world_model/object_tracker.cpp
  humanoid_system/world_model/occupancy_grid.cpp
  humanoid_system/planning/cost_map.cpp
  humanoid_system/planning/constraint_generator.cpp
  humanoid_system/simulation/humanoid_sim.cpp
  humanoid_system/simulation/sensor_sim.cpp
  humanoid_system/simulation/scenario_generator.cpp
  humanoid_system/system/data_bus.cpp
  humanoid_system/system/real_time_loop.cpp
)

target_include_directories(humanoid_system_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if (MSVC)
  target_compile_options(humanoid_system_demo PRIVATE /W4)
else()
  target_compile_options(humanoid_system_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()
