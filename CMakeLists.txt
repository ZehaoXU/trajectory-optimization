cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)

option(traj_opt_build_tests "Build all of trajectoryOptimization's own tests." OFF)
option(traj_opt_build_simulate "Build all of trajectoryOptimization's own tests." ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR) # if building as top-level
	option(traj_opt_build_samples "Build all of trajectoryOptimization's own samples." OFF)
else()
	option(traj_opt_build_samples "Build all of trajectoryOptimization's own samples." OFF)
endif()

project(TrajectoryOptimization VERSION 0.9.1 LANGUAGES CXX)

#Make sure that custom modules like FindIpopt are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

# UNCOMMENT THESE TO ENABLE USING MUJOCO
# Load local dependency paths
include(cmake/LocalProperties.cmake)
find_package(Mujoco REQUIRED MODULE)

find_package(Ipopt REQUIRED MODULE)
#set(IPOPT_LIBRARIES ${IPOPT_DIR}/lib)

find_package(Rangev3 REQUIRED MODULE)

add_library(trajectoryOptimizationLib INTERFACE)
#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(TrajectoryOptimization::TrajectoryOptimizationLib ALIAS trajectoryOptimizationLib)

target_include_directories(trajectoryOptimizationLib INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
	$<INSTALL_INTERFACE:include>
	${IPOPT_DIR}/include
	~/.mujoco/mjpro150/include
)
target_link_libraries(trajectoryOptimizationLib INTERFACE
	Ipopt::Ipopt Rangev3::Rangev3 Mujoco::Mujoco
)

if (traj_opt_build_tests)
	enable_testing()
	add_subdirectory(test)
endif()

if (traj_opt_build_samples)
	add_executable(sample src/trajectoryOptimizationMain.cpp)
	message(${IPOPT_FOUND})
	message(${IPOPT_INCLUDE_DIRS})
	message(${IPOPT_DIR})
	#target_include_directories(trajectoryOptimizationSample PRIVATE ${IPOPT_DIR}/include)
	target_link_libraries(sample
		PUBLIC
			Ipopt::Ipopt Rangev3::Rangev3 Mujoco::Mujoco
		PRIVATE
			trajectoryOptimizationLib
	)
endif()

if (traj_opt_build_simulate)
	add_executable(simulate src/simulate.cpp)
	message(${IPOPT_FOUND})
	message(${IPOPT_INCLUDE_DIRS})
	message(${IPOPT_DIR})
	#target_include_directories(trajectoryOptimizationSample PRIVATE ${IPOPT_DIR}/include)
	target_link_libraries(simulate
		PUBLIC
		        Mujoco::Mujoco
	
	)
endif()
