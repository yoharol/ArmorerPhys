cmake_minimum_required(VERSION 3.21)

project(ArmorerPhys
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
# json is already included in tinygltf as header file
# include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/nlohmann_json.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/spdlog.cmake")

# give up on USD and use gltf for now
# option(LINK_USD "Include USD components" OFF)
# if(PROJECT_IS_TOP_LEVEL)
#   set(LINK_USD ON CACHE BOOL "Include USD components" FORCE)
# endif()
# if(LINK_USD)
#   include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/pxr.cmake")
# endif()

igl_include(copyleft tetgen)

# set a list of targets
set(GL_TARGETS
  OpenGL::GL
  imgui
  glfw
  glad
  Eigen3::Eigen
  igl::core
  tetgen::tetgen
#  nlohmann_json::nlohmann_json
  spdlog::spdlog
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
  "${CMAKE_CURRENT_SOURCE_DIR}/include/ArmorerPhys/data/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/tinygltf/tiny_gltf.cc")


set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} STATIC)
target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${GL_TARGETS})
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