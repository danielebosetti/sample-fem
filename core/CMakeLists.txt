message("    core: ")

add_compile_options(/wd26444)
if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()



file(GLOB_RECURSE MODULE_SOURCES src/*.cpp)
file(GLOB_RECURSE MODULE_HEADERS include/*.h)
#file(GLOB_RECURSE MODULE_SOURCES_TEST test/*.cpp)
set(MODULE_SOURCES_TEST test/test-solve.cpp test/test-main.cpp)
set(MISC_SOURCES_TEST test/test-simple.cpp test/misc-tests.cpp)

source_group("src" FILES ${MODULE_SOURCES})
source_group("include" FILES ${MODULE_HEADERS})

add_library (core ${MODULE_SOURCES} ${MODULE_HEADERS})

target_include_directories(core PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> )

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(core PUBLIC Eigen3::Eigen)

# find_package(fmt CONFIG REQUIRED)
# target_link_libraries(core PRIVATE fmt::fmt fmt::fmt-header-only)

find_package(spdlog CONFIG REQUIRED)
target_link_libraries(core PUBLIC spdlog::spdlog spdlog::spdlog_header_only)

enable_testing()
find_package(GTest REQUIRED)

add_executable( core_test ${MODULE_SOURCES_TEST} )
target_link_libraries( core_test core )
target_link_libraries( core_test GTest::GTest)

add_executable( misc_core_test ${MISC_SOURCES_TEST} )
target_link_libraries( misc_core_test core )
target_link_libraries( misc_core_test GTest::GTest)

# add_executable( log_test test/test-log.cpp )
# target_link_libraries( log_test PRIVATE spdlog::spdlog spdlog::spdlog_header_only)

add_test(core_test_all core_test)
# todo filter tests
#add_test(runUnitTests_1 runUnitTests --gtest_include)
