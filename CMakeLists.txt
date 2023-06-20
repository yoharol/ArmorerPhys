cmake_minimum_required(VERSION 3.21)

project(LearnOpengl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

# find opengl and glfw
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# add glad
add_subdirectory(glad)

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
      glfw
      glad)
endfunction()

add_gl_target(01_triangle)