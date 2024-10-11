message("Linking to OpenUSD")
get_property(importTargets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
find_package(pxr REQUIRED)
get_property(importTargetsAfter DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
list(REMOVE_ITEM importTargetsAfter ${importTargets})
message("[ArmorerPhys] Targets from OpenUSD: ${importTargetsAfter}")

set(USD_LIBS 
    usd
    usdGeom
    usdSkel
    tf
    vt
    gf
    sdf
    arch
    work)