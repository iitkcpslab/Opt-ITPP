project(IntegratedTaskAndMotionPlanner)

cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(Z3_DIR "../z3/build/")

include_directories("CBS-PC/inc")
file(GLOB SOURCES "CBS-PC/src/cbs/*.cpp")
file(GLOB STP_SOURCES "CBS-PC/src/stp/*.cpp")

# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
# Find the Z3 package
find_package(Z3 REQUIRED)
message(STATUS "Z3_FOUND: ${Z3_FOUND}")
message(STATUS "Found Z3 ${Z3_VERSION_STRING}")
message(STATUS "Z3_DIR: ${Z3_DIR}")

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${Z3_INCLUDE_DIRS} )

add_library( MAPF_PC_LIB STATIC ${SOURCES} ${STP_SOURCES} )
add_library( TASK_PLANNER STATIC "src/Graph.cpp" "src/TaskPlanner.cpp" )


add_executable(cbs  "CBS-PC/src/cbs.cpp" )
add_executable(mastar  "CBS-PC/src/runMaStar.cpp" )
add_executable(jsonTester "src/Parser.cpp")
add_executable(taskPlanner  "runTaskPlanner.cpp")
add_executable(itmpPlanner  "src/IntegratedPlanner.cpp" "runITMP.cpp")

set_property(
  TARGET cbs
  PROPERTY COMPILE_DEFINITIONS IL_STD
)

target_link_libraries(cbs MAPF_PC_LIB)
target_link_libraries(mastar MAPF_PC_LIB)
target_link_libraries(taskPlanner TASK_PLANNER)
target_link_libraries(itmpPlanner MAPF_PC_LIB)
target_link_libraries(itmpPlanner TASK_PLANNER)

target_link_libraries(cbs ${Boost_LIBRARIES})
target_link_libraries(mastar ${Boost_LIBRARIES})
target_link_libraries(jsonTester ${Boost_LIBRARIES})
target_link_libraries(taskPlanner ${Boost_LIBRARIES})
target_link_libraries(itmpPlanner ${Boost_LIBRARIES})

target_link_libraries(TASK_PLANNER ${Z3_LIBRARIES})

# target_include_directories(taskPlanner ${Z3_C_INCLUDE_DIRS})
# target_link_libraries(taskPlanner ${Z3_LIBRARIES})