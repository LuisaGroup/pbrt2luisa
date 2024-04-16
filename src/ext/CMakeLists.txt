add_library(minipbrt INTERFACE)
target_include_directories(minipbrt INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/minipbrt>)
add_library(minipbrt-object OBJECT
        minipbrt/minipbrt.cpp
        minipbrt/minipbrt.h)
target_link_libraries(minipbrt-object PUBLIC minipbrt)

add_library(nlohmann-json INTERFACE)
target_include_directories(nlohmann-json INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/json/single_include>)
