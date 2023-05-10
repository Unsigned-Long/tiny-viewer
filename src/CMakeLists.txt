set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/core CORE_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/entity ENTITY_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/object OBJECT_FILES)

add_library(
        ${LIBRARY_NAME} SHARED
        ${CORE_FILES}
        ${ENTITY_FILES}
        ${OBJECT_FILES}
)

target_include_directories(
        ${LIBRARY_NAME} PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        pthread
)

# example & test

add_executable(${PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${PROJECT_NAME}_prog PRIVATE
        ${LIBRARY_NAME}
)
