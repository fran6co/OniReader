cmake_minimum_required(VERSION 2.8)
project(OniReader)

option(BUILD_TESTS "Builds tests" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(JPEG REQUIRED)

set(SOURCE_FILES
    include/OniReader.h
    src/OniReader.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(OniReader SHARED ${SOURCE_FILES})
target_link_libraries(OniReader
    ${JPEG_LIBRARIES}
)

install(TARGETS OniReader DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY "${OniReader_SOURCE_DIR}/include" DESTINATION include)

if (BUILD_TESTS)
    set(GOOGLETEST_ROOT 3rdparty/googletest CACHE STRING "Google Test source root")

    include_directories(
        ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}
        ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/include
    )

    set(GOOGLETEST_SOURCES
        ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest-all.cc
        ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest_main.cc
    )

    foreach(_source ${GOOGLETEST_SOURCES})
        set_source_files_properties(${_source} PROPERTIES GENERATED 1)
    endforeach()

    add_library(googletest ${GOOGLETEST_SOURCES})

    add_executable(unit_tests
        test/main.cpp
        test/test.cpp
    )

    target_link_libraries(unit_tests
        googletest
        pthread
        OniReader
    )

    add_dependencies(unit_tests googletest)

    include(CTest)
    enable_testing()

    add_test(unit ${PROJECT_BINARY_DIR}/unit_tests "${PROJECT_SOURCE_DIR}/data")
endif()