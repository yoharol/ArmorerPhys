
if(NOT glfw3_PKG_FOUND)
  message(STATUS "glfw3 not found, building from source")
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(glfw)
endif()

if(NOT glm_PKG_FOUND)
  message(STATUS "glm not found, building from source")
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  add_subdirectory(glm)
endif()

if(NOT Eigen3_PKG_FOUND)
  message(STATUS "Eigen3 not found, building from source")
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_UNINSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory(eigen)
endif()


set(GLAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glad CACHE INTERNAL "")
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui CACHE INTERNAL "")
