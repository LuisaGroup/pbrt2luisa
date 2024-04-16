add_subdirectory(ext)

add_executable(pbrt2luisa main.cpp)
target_link_libraries(pbrt2luisa PRIVATE
        minipbrt-object
        nlohmann-json)

