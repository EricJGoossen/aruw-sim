# aruw-sim: hw-emulator (with Achilles from its submodule) and the ARUW device
# plugin, in one build. scripts/build.sh drives this, the MCB and otto.
cmake_minimum_required(VERSION 3.21)
project(aruw_sim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif ()

# Achilles from the submodule, not a fetch.
set(FETCHCONTENT_SOURCE_DIR_ACHILLES "${CMAKE_CURRENT_SOURCE_DIR}/achilles" CACHE PATH "Achilles checkout")
option(HWEMU_BUILD_TESTS "Build hw-emulator's tests" OFF)
add_subdirectory(hw-emulator)
add_subdirectory(aruw_devices)
