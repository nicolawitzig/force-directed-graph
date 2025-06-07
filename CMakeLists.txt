cmake_minimum_required(VERSION 3.10)
project(ForceDirectedGraph LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PREFIX_PATH "/opt/homebrew;/usr/local")

# SFML 3.0 uses different component names
find_package(SFML COMPONENTS System Window Graphics REQUIRED)
find_package(Eigen3 3.3 REQUIRED CONFIG)

set(APP_SOURCES
    src/main.cpp
    src/Graph.cpp
    src/Node.cpp
    src/Edge.cpp
)

add_executable(ForceDirectedGraphApp ${APP_SOURCES})

target_include_directories(ForceDirectedGraphApp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# SFML 3.0 uses different library names
target_link_libraries(
    ForceDirectedGraphApp PRIVATE 
    SFML::System SFML::Window SFML::Graphics
    Eigen3::Eigen
    )