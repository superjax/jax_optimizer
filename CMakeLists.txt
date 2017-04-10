cmake_minimum_required(VERSION 2.8.3)
project(jax_optimizer)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11 ${OpenMP_CXX_FLAGS} -O3")

find_package(GTSAM)

include_directories(include ${GTSAM_INCLUDE_DIR})

add_executable(jax_optimizer
	include/jax_optimizer.h
	src/jax_optimizer.cpp)

target_link_libraries(jax_optimizer gtsam)
