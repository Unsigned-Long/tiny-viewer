# Tiny-Viewer: Tiny But Powerful Graphic Entity And Object Visualization
# Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
# https://github.com/Unsigned-Long/tiny-viewer.git
#
# Author: Shuolong Chen (shlchen@whu.edu.cn)
# GitHub: https://github.com/Unsigned-Long
#  ORCID: 0000-0002-5283-9057
#
# Purpose: See .h/.hpp file.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * The names of its contributors can not be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/core CORE_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/entity ENTITY_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/object OBJECT_SRC_FILES)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/tiny-viewer/core CORE_HEADER_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/tiny-viewer/entity ENTITY_HEADER_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/tiny-viewer/object OBJECT_HEADER_FILES)

add_library(
        ${LIBRARY_NAME} SHARED
        ${CORE_SRC_FILES} ${ENTITY_SRC_FILES} ${OBJECT_SRC_FILES}
        ${CORE_HEADER_FILES} ${ENTITY_HEADER_FILES} ${OBJECT_HEADER_FILES}
)

set(ADDITIONAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

target_include_directories(
        ${LIBRARY_NAME} PUBLIC
        # only when building from the source tree
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        # only when using the lib from the install path
        $<INSTALL_INTERFACE:include>
        ${ADDITIONAL_INCLUDE_DIRS}
)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES}
        pthread
)

# example & test

add_executable(${PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${PROJECT_NAME}_prog PRIVATE
        ${LIBRARY_NAME}
)
