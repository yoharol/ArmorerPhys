# integrate glad into a cmake static library
set(GLAD_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/glad/src/glad.c
)
set(GLAD_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/glad/include
)

add_library(glad STATIC ${GLAD_SOURCES})
target_include_directories(glad PUBLIC ${GLAD_INCLUDE_DIRS})
