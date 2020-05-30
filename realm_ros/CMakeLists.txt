cmake_minimum_required(VERSION 3.9)

project(realm_ros)


################################################################################
# Include CMake dependencies
################################################################################


include(CheckLanguage)


################################################################################
# Compiler specific configuration
################################################################################


check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    message(STATUS "CUDA Support enabled.")

    include(FindCUDA)
    set(CUDA_ARCH_LIST Auto CACHE STRING  "List of CUDA architectures (e.g. Pascal, Volta, etc) or \
                                           compute capability versions (6.1, 7.0, etc) to generate code for. \
                                           Set to Auto for automatic detection (default).")
    cuda_select_nvcc_arch_flags(CUDA_ARCH_FLAGS ${CUDA_ARCH_LIST})
    list(APPEND CUDA_NVCC_FLAGS ${CUDA_ARCH_FLAGS})
else()
    message(STATUS "CUDA Support disabled.")
endif()


################################################################################
# Dependencies
################################################################################


find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        pcl_ros
        geodesy
        cv_bridge
        tf
        std_msgs
        nav_msgs
        sensor_msgs
        geographic_msgs
        realm_msgs
)

# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.2 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()

find_package(cmake_modules REQUIRED)

find_package(OpenREALM REQUIRED)


################################################################################
# Catkin Package
################################################################################


catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            roslib
            geodesy
            cv_bridge
            tf
            std_msgs
            nav_msgs
            sensor_msgs
            geographic_msgs
            realm_msgs
            cmake_modules
        DEPENDS
            OpenCV
)


################################################################################
# Build
################################################################################


add_library(${PROJECT_NAME} SHARED
        src/realm_ros_lib/grabber_ros_node.cpp
        src/realm_ros_lib/grabber_exiv2_node.cpp
        src/realm_ros_lib/stage_node.cpp
        src/realm_ros_lib/conversions.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${catkin_LIBRARIES}
        ${cmake_modules_LIBRARIES}
        ${OpenCV_LIBRARIES}
	    realm_stages
)

add_dependencies(${PROJECT_NAME}
        realm_msgs_generate_messages_cpp
)

################################################################################
# Executables
################################################################################


add_executable(realm_ros_grabber src/grabber_ros_node_main.cpp)
add_dependencies(realm_ros_grabber ${catkin_EXPORTED_TARGETS})
target_link_libraries(realm_ros_grabber PUBLIC ${PROJECT_NAME})

add_executable(realm_exiv2_grabber src/grabber_exiv2_node_main.cpp)
add_dependencies(realm_exiv2_grabber ${catkin_EXPORTED_TARGETS})
target_link_libraries(realm_exiv2_grabber PUBLIC ${PROJECT_NAME})

add_executable(realm_stage_node src/stage_node_main.cpp)
add_dependencies(realm_stage_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(realm_stage_node PUBLIC ${PROJECT_NAME} -lCGAL)

## Compile Orb vocabulary
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(bin_vocabulary tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME} ${catkin_LIBRARIES})


################################################################################
# Install
################################################################################


# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
)
