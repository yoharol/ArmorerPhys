cmake_minimum_required(VERSION 3.5)

project(ArmorerPhys
  DESCRIPTION "A simple OpenGL library"
  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

get_filename_component(ABS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
add_definitions(-DASSETS_PATH="${ABS_SOURCE_DIR}/assets")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external)

find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/eigen3.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/glfw.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/glad.cmake")
include ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/imgui.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/libigl.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/spdlog.cmake")

# give up on USD and use gltf for now
set(LINK_USD OFF)
if(LINK_USD)
  add_definitions(-DUSD_ENABLED)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/pxr.cmake")
endif()

igl_include(copyleft tetgen)
igl_include(restricted triangle)

# set a list of targets
set(GL_TARGETS
  OpenGL::GL
  imgui
  glfw
  glad
  Eigen3::Eigen
  igl::core
  tetgen::tetgen
  triangle::triangle
  spdlog::spdlog
  OpenMP::OpenMP_CXX
)

# set(USD_TARGETS
#   usd
#   usdGeom
#   vt
#   usdImaging
#   usdImagingGL
#   )

file(GLOB CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/include/ArmorerPhys/*.cpp")
file(GLOB_RECURSE RENDER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/include/ArmorerPhys/render/*.cpp")
file(GLOB_RECURSE SIM_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/include/ArmorerPhys/sim/*.cpp")
# file(GLOB_RECURSE USD_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/include/ArmorerPhys/USD/*.cpp")
file(
  GLOB_RECURSE DATA_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/include/ArmorerPhys/data/*.cpp")


set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} STATIC)
target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${GL_TARGETS})

if(LINK_USD)
  target_link_libraries(${LIBRARY_NAME} PUBLIC ${USD_LIBS})
endif()

target_sources(${LIBRARY_NAME} PRIVATE ${CORE_SOURCES} ${RENDER_SOURCES} ${SIM_SOURCES} ${DATA_SOURCES})

#if (LINK_USD)
#    target_link_libraries(${LIBRARY_NAME} PUBLIC ${USD_TARGETS})
#   target_sources(${LIBRARY_NAME} PRIVATE ${USD_SOURCES})
# endif()

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

file(GLOB EXAMPLE_SOURCES "examples/*.cpp")

if(PROJECT_IS_TOP_LEVEL)
  foreach(EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
      get_filename_component(EXAMPLE_NAME ${EXAMPLE_SOURCE} NAME_WE)
      add_gl_target(${EXAMPLE_NAME})
  endforeach()
endif()