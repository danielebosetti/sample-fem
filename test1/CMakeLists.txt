message("    test1: ")

add_executable (test1 src/test.cpp)

target_link_libraries(test1 PUBLIC core)
