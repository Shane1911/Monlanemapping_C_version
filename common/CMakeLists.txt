include_directories("/usr/include/eigen3")

add_library(file_io file_flow.cc)
target_link_libraries(file_io
        ${THIRD_PARTY_LIBS}
)

add_library(lib_kdtree kdtree.cc)
target_link_libraries(lib_kdtree
        ${THIRD_PARTY_LIBS}
)

# add_library(lib_ctrl_points ctrl_points.cc)
# target_link_libraries(lib_ctrl_points
#         ${THIRD_PARTY_LIBS}
#         lib_kdtree
#         lib_catmull_spline
# )

# add_library(lib_kmmatcher hungarin.cc)
# target_link_libraries(lib_kmmatcher
#         ${THIRD_PARTY_LIBS}
# )

add_subdirectory(spline)
