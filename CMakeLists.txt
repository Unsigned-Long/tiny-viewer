cmake_minimum_required(VERSION 3.10)

project(tiny-viewer VERSION 1.0)

# -----------
# set lib name ans space
# -----------
set(LIBRARY_NAME tiny-viewer)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# -----------
# for install
# -----------

# Configuration
set(CONFIG_INSTALL_DIR "lib/cmake/${LIBRARY_NAME}")
set(INCLUDE_INSTALL_DIR "include")
set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}ConfigVersion.cmake")
set(PROJ_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${LIBRARY_NAME}Targets")

# Include module with function 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

write_basic_package_version_file("${VERSION_CONFIG}" COMPATIBILITY SameMajorVersion)

configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in
        "${PROJ_CONFIG}"
        INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
)

export(
        TARGETS ${LIBRARY_NAME}
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGETS_EXPORT_NAME}.cmake"
)

install(
        TARGETS ${LIBRARY_NAME}
        EXPORT "${TARGETS_EXPORT_NAME}"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        INCLUDES DESTINATION "${INCLUDE_INSTALL_DIR}"
)

install(
        FILES "${PROJ_CONFIG}" "${VERSION_CONFIG}"
        DESTINATION "${CONFIG_INSTALL_DIR}"
)

install(
        EXPORT "${TARGETS_EXPORT_NAME}"
        NAMESPACE "${namespace}"
        DESTINATION "${CONFIG_INSTALL_DIR}"
)

install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/include/${LIBRARY_NAME}
        DESTINATION ${INCLUDE_INSTALL_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN "*.hpp"
)