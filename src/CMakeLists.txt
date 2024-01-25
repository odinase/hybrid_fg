add_executable(main
    main.cpp
)

target_link_libraries(main PUBLIC
    spdlog::spdlog
    range-v3::range-v3
    Eigen3::Eigen
    gtsam
)

target_include_directories(main
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(TARGETS main
    DESTINATION ${CMAKE_SOURCE_DIR}
)