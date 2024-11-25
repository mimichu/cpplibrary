cmake_minimum_required(VERSION 3.10)
project(RobotUtilities)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include/)

# Find Eigen3
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)
message(STATUS "yaml-cpp include dir: ${YAML_CPP_INCLUDE_DIR}")

###########
## Build ##
###########
add_library(Utilities SHARED
    src/spatial_utilities.cpp
    src/timer_linux.cpp
    src/butterworth.cpp
)

target_link_libraries(Utilities 
    PUBLIC 
    Eigen3::Eigen
    yaml-cpp
)

target_include_directories(Utilities
    PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)

add_executable(test_utilities src/test.cpp)
target_link_libraries(test_utilities Utilities)

#############
## Install ##
#############
# headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)

# binaries
install(TARGETS Utilities
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/
)