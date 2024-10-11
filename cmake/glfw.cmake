if(TARGET glfw)
    return()
else()
  find_package(glfw3 QUIET)
  if(TARGET glfw)
    message(STATUS "[ArmorerPhys] Third-party: glfw3 loaded")
    return()
  endif()
endif()

message(STATUS "[ArmorerPhys] Third-party: glfw3 not found, fetching it with git")

include(FetchContent)
FetchContent_Declare(
    glfw3
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8
    GIT_SHALLOW TRUE
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw3)

