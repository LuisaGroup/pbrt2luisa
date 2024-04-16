#include "logging.h"
#include "convert.h"

int main(int argc, char *argv[]) {
    if (argc == 1) {
        luisa::println("Usage: {} <scene.pbrt>", argv[0]);
    } else {
        luisa::render::convert(argv[1]);
    }
}
