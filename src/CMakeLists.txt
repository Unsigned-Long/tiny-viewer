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

set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

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
