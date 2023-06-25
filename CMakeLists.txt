cmake_minimum_required(VERSION 3.21)

project(LearnOpengl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

get_filename_component(ABS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
add_definitions(-DASSETS_PATH="${ABS_SOURCE_DIR}/assets")


function(find_package_with_targets package)
	get_property(importTargets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
	find_package(${package} REQUIRED)
  get_property(importTargetsAfter DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
  list(REMOVE_ITEM importTargetsAfter ${importTargets})
	message("Targets from ${package}: ${importTargetsAfter}")
endfunction()

# find opengl and glfw
find_package_with_targets(OpenGL)
find_package_with_targets(glfw3)
find_package_with_targets(Eigen3)
find_package_with_targets(glm)

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.4.0
)
FetchContent_MakeAvailable(libigl)

# add glad
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/glad.cmake")
include ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/imgui.cmake")

# glob all cpp files in src folder
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# a cmake function that create an executable target with opengl linked
function(add_gl_target target_name)
    add_executable(${target_name})
    target_sources(${target_name}
      PRIVATE
      ${SOURCES}
      ${target_name}.cpp)
    target_include_directories(${target_name}
      PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${target_name}
      PRIVATE
      OpenGL::GL
      Eigen3::Eigen
      igl::core
      glfw
      glad)
endfunction()

add_gl_target(01_triangle)
add_gl_target(02_model)
add_gl_target(03_lighting)
add_gl_target(04_pipeline)