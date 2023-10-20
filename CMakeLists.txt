cmake_minimum_required(VERSION 3.21)

project(ArmorerGL
  DESCRIPTION "A simple OpenGL library"
  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

get_filename_component(ABS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
add_definitions(-DASSETS_PATH="${ABS_SOURCE_DIR}/assets")

function(find_package_with_targets package)
	get_property(importTargets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
	find_package(${package} QUIET)
  message("Package ${package} found: ${${package}_FOUND}")
  set(${package}_PKG_FOUND ${${package}_FOUND} PARENT_SCOPE)
  get_property(importTargetsAfter DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
  list(REMOVE_ITEM importTargetsAfter ${importTargets})
	message("Targets from ${package}: ${importTargetsAfter}")
endfunction()

# find opengl and glfw
find_package_with_targets(OpenGL)
find_package_with_targets(Eigen3)
find_package_with_targets(glfw3)
find_package_with_targets(glm)


add_subdirectory(external)

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.4.0
)
FetchContent_MakeAvailable(libigl)

# add glad and imgui
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/glad.cmake")
include ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/imgui.cmake")

# add all header files in /include into a header only library
set(LIBRARY_NAME ArmorerGL)
add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${LIBRARY_NAME} INTERFACE OpenGL::GL Eigen3::Eigen igl::core imgui glfw glad)

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
      ${LIBRARY_NAME})
endfunction()

if(PROJECT_IS_TOP_LEVEL)
  add_gl_target(01_triangle)
  add_gl_target(02_model)
  add_gl_target(03_lighting)
  add_gl_target(04_pipeline)
  add_gl_target(05_2d_game)
  add_gl_target(06_handle_input)
  add_gl_target(07_imgui_input)
  add_gl_target(08_Affine2d)
  add_gl_target(09_mathplot2d)
endif()

