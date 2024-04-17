add_subdirectory(ext)

add_executable(pbrt2luisa
        main.cpp
        logging.h
        convert.cpp
        convert.h)

target_link_libraries(pbrt2luisa PRIVATE
        minipbrt-object
        nlohmann-json
        magic_enum
        glm::glm-header-only
        fmt::fmt-header-only)
