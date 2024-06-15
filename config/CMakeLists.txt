add_library(file_config config_gflags.cc)
target_link_libraries(file_config
        ${THIRD_PARTY_LIBS}
)
