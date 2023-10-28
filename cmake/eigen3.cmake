if(TARGET Eigen3::Eigen)
    return()
else()
  find_package(Eigen3 QUIET)
  if(TARGET Eigen3::Eigen)
    message(STATUS "[ArmorerGL] Third-party: Eigen3::Eigen loaded")
    return()
  endif()
endif()

message(STATUS "[ArmorerGL] Third-party: Eigen3::Eigen not found, fetching it with git")

include(FetchContent)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.3.9
    GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_UNINSTALL OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF)
FetchContent_MakeAvailable(eigen)

