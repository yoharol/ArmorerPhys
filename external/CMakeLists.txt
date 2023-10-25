
if(NOT TARGET glfw)
  message(STATUS "[ArmorerGL] glfw3 not found, building from source")
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(glfw)
endif()

if(NOT TARGET Eigen3::Eigen)
  message(STATUS "[ArmorerGL] Eigen3 not found, building from source")
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_UNINSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory(eigen)
endif()


set(GLAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glad CACHE INTERNAL "")
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui CACHE INTERNAL "")
