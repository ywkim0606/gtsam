# Install headers
set(subdir sparse)
file(GLOB sparse "*.h")
# FIXME: exclude headers
install(FILES ${sparse_headers} DESTINATION include/gtsam/sparse)

# Add all tests
add_subdirectory(tests)