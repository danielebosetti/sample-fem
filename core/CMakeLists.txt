message("    core: ")

file(GLOB_RECURSE MODULE_SOURCES src/*.cpp)
file(GLOB_RECURSE MODULE_HEADERS include/*.h)

source_group("src" FILES ${MODULE_SOURCES})
source_group("include" FILES ${MODULE_HEADERS})

add_library (core ${MODULE_SOURCES} ${MODULE_HEADERS})

target_include_directories(core PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> )

find_package(Eigen3 CONFIG REQUIRED)

target_link_libraries(core PUBLIC Eigen3::Eigen)
