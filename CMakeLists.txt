cmake_minimum_required(VERSION 3.21)

project(ArmorerGL
  DESCRIPTION "A simple OpenGL library"
  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

get_filename_component(ABS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
add_definitions(-DASSETS_PATH="${ABS_SOURCE_DIR}/assets")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external)

find_package(OpenGL REQUIRED)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/eigen3.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/glfw.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/glad.cmake")
include ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/imgui.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/libigl.cmake")

option(ARMORER_USD "Include USD components" OFF)
if(PROJECT_IS_TOP_LEVEL)
  set(ARMORER_USD ON CACHE BOOL "Include USD components" FORCE)
endif()
if(ARMORER_USD)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/pxr.cmake")
endif()

# set a list of targets
set(GL_TARGETS
  OpenGL::GL
  imgui
  glfw
  glad
  Eigen3::Eigen
  igl::core
  )

set(USD_TARGETS
  usd
  usdGeom
  vt
  usdImaging
  usdImagingGL
  )

set(LIBRARY_NAME ArmorerGL)
add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${LIBRARY_NAME} INTERFACE ${GL_TARGETS})
if (LINK_USD)
  target_link_libraries(${LIBRARY_NAME} INTERFACE ${USD_TARGETS})
endif()

# a cmake function that create an executable target with opengl linked
function(add_gl_target target_name)
    add_executable(${target_name})
    target_sources(${target_name}
      PRIVATE
      ${SOURCES}
      ${CMAKE_CURRENT_SOURCE_DIR}/examples/${target_name}.cpp)
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

