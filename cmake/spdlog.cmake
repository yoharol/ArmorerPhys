if(NOT TARGET spdlog::spdlog)
  message("[ArmorerPhys] Fetching spdlog")
  FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG        v1.14.0
  )
  FetchContent_MakeAvailable(spdlog)
endif()
