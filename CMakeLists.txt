# This file is part of OpenREALM.
#
# Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
# For more information see <https://github.com/laxnpander/OpenREALM>
#
# OpenREALM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# OpenREALM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.9)

project(OpenREALM_ROS1_Bridge)

set(OpenREALM_ROS1_BRIDGE_VERSION "1.0")
set(OpenREALM_ROS1_BRIDGE_VERSION_NUMBER "1000")


################################################################################
# Include CMake dependencies
################################################################################

################################################################################
# Options
################################################################################


################################################################################
# Compiler specific configuration
################################################################################


cmake_policy(SET CMP0028 NEW)

if(CMAKE_BUILD_TYPE)
    message(STATUS "Build type specified as ${CMAKE_BUILD_TYPE}")
else()
    message(STATUS "Build type not specified, using Release")
    set(CMAKE_BUILD_TYPE Release)
    set(IS_DEBUG OFF)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


################################################################################
# Add modules
################################################################################


add_subdirectory(realm_msgs)
add_subdirectory(realm_ros)
add_subdirectory(realm_rviz_plugin)


################################################################################
# Done :)
################################################################################
