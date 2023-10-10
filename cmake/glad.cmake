# integrate glad into a cmake static library
message("GLAD_DIR: ${GLAD_DIR}")

set(GLAD_SOURCES
    ${GLAD_DIR}/src/glad.c
)
set(GLAD_INCLUDE_DIRS
    ${GLAD_DIR}/include
)

add_library(glad STATIC ${GLAD_SOURCES})
target_include_directories(glad PUBLIC ${GLAD_INCLUDE_DIRS})
