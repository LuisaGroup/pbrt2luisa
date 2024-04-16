//
// Created by Mike on 2024/4/16.
//

#include <unordered_map>
#include <filesystem>

#include <nlohmann/json.hpp>
#include <magic_enum/magic_enum.hpp>
#include <minipbrt.h>

#include "logging.h"
#include "convert.h"

namespace luisa::render {

[[nodiscard]] static nlohmann::json convert_transform(
    const minipbrt::Scene *scene,
    const minipbrt::Transform &transform) noexcept {
    // TODO: consider animated transform
    auto m = nlohmann::json::array();
    for (auto &&row : transform.start) {
        for (auto x : row) {
            m.emplace_back(x);
        }
    }
    return {
        {"impl", "Matrix"},
        {"prop", {{"m", m}}}};
}

static void dump_mesh_to_wavefront_obj(
    const std::filesystem::path &file_name,
    const minipbrt::TriangleMesh *mesh) noexcept {
    static std::string buffer;
    buffer.resize(1024u * 1024u - 1u);
    buffer.clear();

}

static void convert_shapes(
    const std::filesystem::path &base_dir,
    const minipbrt::Scene *scene,
    nlohmann::json &converted) {
    auto mesh_dir = base_dir / "lr_exported_meshes";
    std::filesystem::create_directories(mesh_dir);
    for (auto shape_index = 0u; shape_index < scene->shapes.size(); shape_index++) {
        auto base_shape = scene->shapes[shape_index];
        auto shape = nlohmann::json::object();
        shape["type"] = "Shape";
        auto &prop = (shape["prop"] = nlohmann::json::object());
        // transform
        prop["transform"] = convert_transform(scene, base_shape->shapeToWorld);
        // surface
        if (auto m = base_shape->material; m != minipbrt::kInvalidIndex) {
            prop["surface"] = std::format("@Surface:{}:{}", m, scene->materials[m]->name);
        }
        // light
        if (auto l = base_shape->areaLight; l != minipbrt::kInvalidIndex) {
            prop["light"] = std::format("@Light:{}", l);
        }
        switch (auto shape_type = base_shape->type()) {
            case minipbrt::ShapeType::Sphere: {
                auto sphere = static_cast<const minipbrt::Sphere *>(base_shape);
                shape["impl"] = "Sphere";
                prop["radius"] = sphere->radius;
                prop["subdivision"] = 4;
                break;
            }
            case minipbrt::ShapeType::TriangleMesh: {
                auto mesh = static_cast<const minipbrt::TriangleMesh *>(base_shape);
                dump_mesh_to_wavefront_obj(mesh_dir / std::format("{:05}.obj", shape_index), mesh);
                shape["impl"] = "Mesh";
                prop["file"] = std::format("lr_exported_meshes/{:05}.obj", shape_index);
                if (mesh->alpha != minipbrt::kInvalidIndex) { eprintln("Ignored unsupported shape alpha at index {}.", shape_index); }
                if (mesh->shadowalpha != minipbrt::kInvalidIndex) { eprintln("Ignored unsupported shape shadow alpha at index {}.", shape_index); }
                break;
            }
            default: eprintln("Ignored unsupported shape at index {} with type '{}'.",
                              shape_index, magic_enum::enum_name(shape_type));
        }
        if (shape.contains("impl")) {
            converted[std::format("Shape:{}", shape_index)] = shape;
        }
    }
}

static void convert_scene(const std::filesystem::path &source_path,
                          const minipbrt::Scene *scene) noexcept {
    try {
        println("Time: {} -> {}", scene->startTime, scene->endTime);
        println("Medium count: {}", scene->mediums.size());
        auto base_dir = source_path.parent_path();
        auto converted = nlohmann::json::object();
        convert_shapes(base_dir, scene, converted);
        // TODO
        println("Converted:\n{}", converted.dump(4).c_str());
    } catch (const std::exception &e) {
        luisa::panic("{}", e.what());
    }
}

void convert(const char *scene_file_name) noexcept {
    try {
        auto scene_file = std::filesystem::canonical(scene_file_name);
        minipbrt::Loader loader;
        if (loader.load(scene_file.string().c_str())) {
            minipbrt::Bits<minipbrt::ShapeType> shape_types;
            shape_types.set(minipbrt::ShapeType::Nurbs);
            shape_types.set(minipbrt::ShapeType::LoopSubdiv);
            shape_types.set(minipbrt::ShapeType::HeightField);
            shape_types.set(minipbrt::ShapeType::PLYMesh);
            if (loader.borrow_scene()->shapes_to_triangle_mesh(shape_types)) {
                convert_scene(scene_file, loader.borrow_scene());
            } else {
                luisa::panic("Failed to load all PLY meshes");
            }
        } else {
            auto e = loader.error();
            auto message = e ? std::format("{} [{}:{}:{}]",
                                           e->message(), e->filename(), e->line(), e->column()) :
                               "unknown";
            luisa::panic("Failed to load scene file {}: {}", scene_file.string(), message);
        }
    } catch (const std::exception &e) {
        luisa::panic("{}", e.what());
    }
}

}// namespace luisa::render