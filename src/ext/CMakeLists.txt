add_library(minipbrt INTERFACE)
target_include_directories(minipbrt INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/minipbrt>)
add_library(minipbrt-object OBJECT
        minipbrt/minipbrt.cpp
        minipbrt/minipbrt.h)
target_link_libraries(minipbrt-object PUBLIC minipbrt)

add_library(nlohmann-json INTERFACE)
target_include_directories(nlohmann-json INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/json/single_include>)

add_library(magic_enum INTERFACE)
target_include_directories(magic_enum INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/magic_enum/include>)

add_subdirectory(glm)
