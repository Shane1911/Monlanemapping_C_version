include_directories("/usr/include/eigen3")

add_library(lib_catmull_spline  catmullrom_spline.cc)
target_link_libraries(lib_catmull_spline
        ${THIRD_PARTY_LIBS}
)

add_library(lib_catmull_splinelist  catmullrom_splinelists.cc)
target_link_libraries(lib_catmull_splinelist
        ${THIRD_PARTY_LIBS}
        lib_catmull_spline
)

add_library(lib_centercatmull_spline  centerpetcatmullrom_spline.cc)
target_link_libraries(lib_centercatmull_spline
        ${THIRD_PARTY_LIBS}
)
