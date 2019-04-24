cmake_minimum_required(VERSION 3.10)
project(Tracker)

set(CMAKE_CXX_STANDARD 11)

add_executable(Tracker main.cpp tracker.cpp tracker.h kalmanFilter.cpp kalmanFilter.h hungarian.cpp hungarian.h)