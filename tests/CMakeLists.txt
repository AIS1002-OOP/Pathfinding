
include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("${CMAKE_SOURCE_DIR}/examples")

add_executable(testCoordinate TestCoordinate.cpp)
add_test(NAME testCoordinate COMMAND testCoordinate)

add_executable(testAstar TestAStarAlgorithm.cpp)
add_test(NAME testAstar COMMAND testAstar)