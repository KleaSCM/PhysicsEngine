file(GLOB_RECURSE SOURCES "*.cpp")

add_library(PhysicsEngine STATIC ${SOURCES})
target_include_directories(PhysicsEngine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
