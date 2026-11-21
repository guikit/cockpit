cmake_minimum_required(VERSION 3.21)
project(guikit_cockpit VERSION 1.0.0 LANGUAGES CXX)

if (PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(GUIKIT_COCKPIT_BUILD_TESTS "Build GuiKit Cockpit unit tests" ${PROJECT_IS_TOP_LEVEL})
option(GUIKIT_COCKPIT_INSTALL "Enable CMake install targets for guikit_cockpit" ${PROJECT_IS_TOP_LEVEL})

if (GUIKIT_COCKPIT_BUILD_TESTS)
    enable_testing()
endif()

if (NOT TARGET guikit::core)
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/guikit/CMakeLists.txt")
        add_subdirectory(third_party/guikit EXCLUDE_FROM_ALL)
    else()
        find_package(guikit REQUIRED)
    endif()
endif()

add_library(guikit_cockpit STATIC
    src/guikit/cockpit/attitude_view.cpp
    src/guikit/cockpit/car_cluster_view.cpp
)
add_library(guikit::cockpit ALIAS guikit_cockpit)
set_target_properties(guikit_cockpit PROPERTIES EXPORT_NAME cockpit)

target_compile_features(guikit_cockpit PUBLIC cxx_std_20)
target_include_directories(guikit_cockpit PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(guikit_cockpit PUBLIC guikit::core)

if (COMMAND guikit_enforce_guidelines)
    guikit_enforce_guidelines(guikit_cockpit
        HEADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include"
        STANDALONE_HEADERS ${GUIKIT_COCKPIT_BUILD_TESTS}
        TEST_NAME guikit_cockpit_header_purity_scan
    )
endif()

if (GUIKIT_COCKPIT_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    install(TARGETS guikit_cockpit
        EXPORT guikitCockpitTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/guikit"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    )
    install(EXPORT guikitCockpitTargets
        FILE guikitCockpitTargets.cmake
        NAMESPACE guikit::
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guikit_cockpit"
    )
    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/guikit_cockpitConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_cockpitConfig.cmake"
        INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guikit_cockpit"
    )
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_cockpitConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_cockpitConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_cockpitConfigVersion.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guikit_cockpit"
    )
endif()

if (GUIKIT_COCKPIT_BUILD_TESTS AND TARGET guikit::guikit)
    add_executable(test_cockpit
        tests/test_cockpit.cpp
    )
    target_link_libraries(test_cockpit PRIVATE guikit::cockpit guikit::guikit)
    if (TARGET guikit::sdl)
        target_link_libraries(test_cockpit PRIVATE guikit::sdl)
    endif()
    if (COMMAND guikit_enforce_guidelines)
        guikit_enforce_guidelines(test_cockpit)
    endif()
    add_test(NAME cockpit COMMAND test_cockpit)
endif()
