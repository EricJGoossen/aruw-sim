# ARUW-specific hub devices, built as a plugin the hub loads (devices.yaml
# `plugins:`). Links the shared libhwemu, the same copy the hub uses.
add_library(aruw_devices MODULE aruw_devices.cpp)
target_link_libraries(aruw_devices PRIVATE hwemu)
target_compile_options(aruw_devices PRIVATE -Wall -Wextra)
set_target_properties(aruw_devices PROPERTIES PREFIX "lib" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins")
