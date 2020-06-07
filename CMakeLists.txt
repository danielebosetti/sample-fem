cmake_minimum_required(VERSION 3.17)

set(CMAKE_CXX_STANDARD 17)

project(sample-fem)

message("    main:")

add_subdirectory(core)

# 
set(VS_STARTUP_PROJECT core_test)
