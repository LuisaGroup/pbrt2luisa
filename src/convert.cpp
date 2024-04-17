//
// Created by Mike on 2024/4/16.
//

#include <fstream>
#include <filesystem>
#include <numbers>

#include <nlohmann/json.hpp>
#include <magic_enum/magic_enum.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <minipbrt.h>

#include "logging.h"
#include "convert.h"

namespace luisa::render {

[[nodiscard]] constexpr auto degrees(double x) noexcept {
    return x / std::numbers::pi * 180.;
}

[[nodiscard]] constexpr auto radians(double x) noexcept {
    return x * std::numbers::pi / 180.;
}

[[nodiscard]] static nlohmann::json convert_transform(const minipbrt::Transform &transform) noexcept {
    // TODO: consider animated transform
    // check if is identity
    float identity[4][4]{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    };
    auto is_identity = true;
    for (auto i = 0; i < 4; i++) {
        for (auto j = 0; j < 4; j++) {
            if (transform.start[i][j] != identity[i][j]) {
                is_identity = false;
                break;
            }
        }
    }
    if (is_identity) { return nullptr; }
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

[[nodiscard]] static nlohmann::json convert_camera_transform(const minipbrt::Transform &transform) noexcept {
    // TODO: consider animated transform
    glm::mat4 m;
    for (auto i = 0; i < 4; i++) {
        for (auto j = 0; j < 4; j++) {
            m[i][j] = transform.start[j][i];
        }
    }
    auto transform_point = [&](glm::vec3 p) noexcept {
        return glm::vec3(m * glm::vec4(p, 1.f));
    };
    auto transform_normal = [&](glm::vec3 n) noexcept {
        auto mm = glm::mat3(m);
        return glm::normalize(glm::transpose(glm::inverse(mm)) * n);
    };
    auto eye = transform_point(glm::vec3(0.f));
    auto up = transform_normal(glm::vec3(0.f, 1.f, 0.f));
    auto front = transform_normal(glm::vec3(0.f, 0.f, 1.f));
    auto right = transform_normal(glm::vec3(1.f, 0.f, 0.f));
    nlohmann::json t{
        {"impl", "View"},
        {"prop",
         {{"origin", {eye.x, eye.y, eye.z}},
          {"front", {front.x, front.y, front.z}},
          {"up", {up.x, up.y, up.z}}}}};
    if (glm::dot(glm::cross(front, right), up) <= 0) {// right handed as we wanted
        return t;
    }
    nlohmann::json s{
        {"impl", "SRT"},
        {"prop", {{"scale", {-1., 1., 1.}}}}};
    return {
        {"impl", "Stack"},
        {"prop",
         {{"transforms", {std::move(s), std::move(t)}}}}};
}

[[nodiscard]] static nlohmann::json convert_envmap_transform(const minipbrt::Transform &transform) noexcept {
    glm::mat4 m;
    for (auto i = 0; i < 4; i++) {
        for (auto j = 0; j < 4; j++) {
            m[i][j] = transform.start[j][i];
        }
    }
    auto n = glm::mat3(m) *
             glm::mat3(glm::rotate(glm::mat4(1.f), -.5f * std::numbers::pi_v<float>, glm::vec3(0, 0, 1))) *
             glm::mat3(glm::scale(glm::mat4(1.f), glm::vec3(1, -1, 1))) *
             glm::mat3(glm::rotate(glm::mat4(1.f), .5f * std::numbers::pi_v<float>, glm::vec3(1, 0, 0)));
    return {
        {"impl", "Matrix"},
        {"prop", {{"m", {n[0][0], n[1][0], n[2][0], 0, n[0][1], n[1][1], n[2][1], 0, n[0][2], n[1][2], n[2][2], 0, 0, 0, 0, 1}}}}};
}

static void dump_mesh_to_wavefront_obj(
    const std::filesystem::path &file_name,
    const minipbrt::TriangleMesh *mesh) {
    std::ofstream f{file_name};
    f << "# Converted from PLY mesh\n";
    for (auto v = 0u; v < mesh->num_vertices; v++) {
        f << luisa::format("v {} {} {}\n",
                           mesh->P[v * 3 + 0],
                           mesh->P[v * 3 + 1],
                           mesh->P[v * 3 + 2]);
    }
    if (mesh->N) {
        for (auto v = 0u; v < mesh->num_vertices; v++) {
            f << luisa::format("vn {} {} {}\n",
                               mesh->N[v * 3 + 0],
                               mesh->N[v * 3 + 1],
                               mesh->N[v * 3 + 2]);
        }
    }
    if (mesh->uv) {
        for (auto v = 0u; v < mesh->num_vertices; v++) {
            f << luisa::format("vt {} {}\n",
                               mesh->uv[v * 2 + 0],
                               mesh->uv[v * 2 + 1]);
        }
    }
    expect(mesh->indices, "Mesh indices are null.");
    expect(mesh->num_indices % 3 == 0, "Invalid number of indices.");
    auto p = [&]() noexcept -> void (*)(std::ofstream &, int, int, int) {
        if (mesh->N) {
            if (mesh->uv) {
                return [](std::ofstream &f, int i0, int i1, int i2) {
                    f << luisa::format("f {}/{}/{} {}/{}/{} {}/{}/{}\n",
                                       i0, i0, i0,
                                       i1, i1, i1,
                                       i2, i2, i2);
                };
            }
            return [](std::ofstream &f, int i0, int i1, int i2) {
                f << luisa::format("f {}//{} {}//{} {}//{}\n",
                                   i0, i0,
                                   i1, i1,
                                   i2, i2);
            };
        }
        if (mesh->uv) {
            return [](std::ofstream &f, int i0, int i1, int i2) {
                f << luisa::format("f {}/{} {}/{} {}/{}\n",
                                   i0, i0,
                                   i1, i1,
                                   i2, i2);
            };
        }
        return [](std::ofstream &f, int i0, int i1, int i2) {
            f << luisa::format("f {} {} {}\n", i0, i1, i2);
        };
    }();
    for (auto i = 0u; i < mesh->num_indices; i += 3) {
        auto i0 = mesh->indices[i + 0] + 1;
        auto i1 = mesh->indices[i + 1] + 1;
        auto i2 = mesh->indices[i + 2] + 1;
        p(f, i0, i1, i2);
    }
}

[[nodiscard]] static std::string material_name(const minipbrt::Scene *scene, uint32_t index) noexcept {
    expect(index != minipbrt::kInvalidIndex, "Invalid material index.");
    auto name = scene->materials[index]->name;
    return luisa::format("Surface:{}:{}", index, name ? name : "unnamed");
}

[[nodiscard]] static std::string texture_name(const minipbrt::Scene *scene, uint32_t index) noexcept {
    expect(index != minipbrt::kInvalidIndex, "Invalid texture index.");
    auto name = scene->textures[index]->name;
    return luisa::format("Texture:{}:{}", index, name ? name : "unnamed");
}

static void color_tex_parsing(
    const minipbrt::Scene *scene,
    nlohmann::json &prop,
    const std::string &name,
    const minipbrt::ColorTex &tex) {
    if (tex.texture == minipbrt::kInvalidIndex) {
        prop[name] = nlohmann::json::object(
            {{"type", "Texture"},
             {"impl", "Constant"},
             {"prop",
              {{"v", nlohmann::json::array({
                         tex.value[0],
                         tex.value[1],
                         tex.value[2],
                     })}}}});
    } else {
        prop[name] = "@" + texture_name(scene, tex.texture);
    }
}

static void float_tex_parsing(const minipbrt::Scene *scene,
                              nlohmann::json &prop,
                              const std::string &name,
                              const minipbrt::FloatTex &tex) {
    if (tex.texture == minipbrt::kInvalidIndex) {
        prop[name] = nlohmann::json::object(
            {{"type", "Texture"},
             {"impl", "Constant"},
             {"prop", {{"v", tex.value}}}});
    } else {
        prop[name] = "@" + texture_name(scene, tex.texture);
    }
}

static void concat_tex_parsing(const minipbrt::Scene *scene,
                               nlohmann::json &prop,
                               const std::string &name,
                               const std::vector<const minipbrt::FloatTex *> &textures) {
    auto channels = nlohmann::json::array();
    for (auto tex : textures) {
        auto temp = nlohmann::json::object({});
        float_tex_parsing(scene, temp, "value", *tex);
        channels.emplace_back(std::move(temp["value"]));
    }
    prop[name] = nlohmann::json::object(
        {{"type", "Texture"},
         {"impl", "Concat"},
         {"prop", {{"channels", channels}}}});
}

static void metal_eta_k_parsing(const minipbrt::Scene *scene,
                                nlohmann::json &prop,
                                const std::string &name,
                                const minipbrt::ColorTex &eta_tex,
                                const minipbrt::ColorTex &k_tex) {
    //    expect(eta_tex.texture != minipbrt::kInvalidIndex, "Invalid eta texture index.");
    //    expect(k_tex.texture != minipbrt::kInvalidIndex, "Invalid k texture index.");
    eprintln("Unsupported metal eta/k parsing.");
}

static void convert_shapes(
    const std::filesystem::path &base_dir,
    const minipbrt::Scene *scene,
    nlohmann::json &converted) {
    auto mesh_dir = base_dir / "lr_exported_meshes";
    std::filesystem::create_directories(mesh_dir);
    // process shapes
    for (auto shape_index = 0u; shape_index < scene->shapes.size(); shape_index++) {
        auto base_shape = scene->shapes[shape_index];
        if (base_shape->insideMedium != minipbrt::kInvalidIndex ||
            base_shape->outsideMedium != minipbrt::kInvalidIndex) {
            eprintln("Ignored unsupported shape inside medium at index {}.", shape_index);
        }
        if (base_shape->reverseOrientation) {
            eprintln("Ignored unsupported shape reverse orientation at index {}.", shape_index);
        }
        auto shape = nlohmann::json::object();
        shape["type"] = "Shape";
        auto &prop = (shape["prop"] = nlohmann::json::object());
        // transform
        if (auto t = convert_transform(base_shape->shapeToWorld); !t.is_null()) {
            prop["transform"] = std::move(t);
        }
        // surface
        if (auto m = base_shape->material; m != minipbrt::kInvalidIndex) {
            prop["surface"] = luisa::format("@{}", material_name(scene, m));
        }
        // light
        if (auto l = base_shape->areaLight; l != minipbrt::kInvalidIndex) {
            prop["light"] = luisa::format("@AreaLight:{}", l);
        }
        switch (auto shape_type = base_shape->type()) {
            case minipbrt::ShapeType::Sphere: {
                auto sphere = static_cast<const minipbrt::Sphere *>(base_shape);
                shape["impl"] = "Instance";
                prop["shape"] = {
                    {"impl", "Sphere"},
                    {"prop",
                     {{"subdivision", 4},
                      {"transform",
                       {{"impl", "SRT"},
                        {"prop",
                         {{"scale", sphere->radius}}}}}}}};
                break;
            }
            case minipbrt::ShapeType::TriangleMesh: {
                auto mesh = static_cast<const minipbrt::TriangleMesh *>(base_shape);
                println("Converting triangle mesh at index {} to Wavefront OBJ.", shape_index);
                dump_mesh_to_wavefront_obj(mesh_dir / luisa::format("{:05}.obj", shape_index), mesh);
                shape["impl"] = "Mesh";
                prop["file"] = luisa::format("lr_exported_meshes/{:05}.obj", shape_index);
                if (auto a = mesh->alpha; a != minipbrt::kInvalidIndex) {// override the material's alpha
                    auto alpha_texture_name = texture_name(scene, a);
                    if (auto m = mesh->material; m == minipbrt::kInvalidIndex) {
                        auto alpha_surface_name = luisa::format("Alpha:{}", alpha_texture_name);
                        if (!converted.contains(alpha_surface_name)) {
                            converted[alpha_surface_name] = {
                                {"type", "Surface"},
                                {"impl", "Matte"},
                                {"prop",
                                 {{"alpha", luisa::format("@{}", alpha_texture_name)}}}};
                        }
                        prop["surface"] = luisa::format("@{}", alpha_surface_name);
                    } else {
                        auto base_surface_name = material_name(scene, m);
                        auto alpha_surface_name = luisa::format("{}:Alpha:{}", base_surface_name, alpha_texture_name);
                        if (!converted.contains(alpha_surface_name)) {
                            auto s = converted[base_surface_name];
                            s["prop"]["alpha"] = luisa::format("@{}", alpha_texture_name);
                            converted[alpha_surface_name] = std::move(s);
                        }
                        prop["surface"] = luisa::format("@{}", alpha_surface_name);
                    }
                }
                break;
            }
            default: eprintln("Ignored unsupported shape at index {} with type '{}'.",
                              shape_index, magic_enum::enum_name(shape_type));
        }
        if (shape.contains("impl")) {
            converted[luisa::format("Shape:{}", shape_index)] = shape;
            if (base_shape->object == minipbrt::kInvalidIndex) {// directly visible shape
                converted["render"]["shapes"].emplace_back(luisa::format("@Shape:{}", shape_index));
            }
        }
    }
    // process objects
    for (auto object_index = 0u; object_index < scene->objects.size(); object_index++) {
        auto base_object = scene->objects[object_index];
        auto object = nlohmann::json::object();
        object["type"] = "Shape";
        object["impl"] = "Group";
        auto &prop = (object["prop"] = nlohmann::json::object());
        // transform
        if (auto t = convert_transform(base_object->objectToInstance); !t.is_null()) {
            prop["transform"] = std::move(t);
        }
        auto &shapes = (prop["shapes"] = nlohmann::json::array());
        if (base_object->firstShape == minipbrt::kInvalidIndex) {
            expect(base_object->numShapes == 0u, "Invalid number of shapes.");
            eprintln("Ignored empty object at index {}.", object_index);
        } else {
            for (auto s = 0u; s < base_object->numShapes; s++) {
                shapes.emplace_back(luisa::format("@Shape:{}", base_object->firstShape + s));
            }
            converted[luisa::format("Object:{}", object_index)] = object;
        }
    }
    // process instances
    for (auto instance_index = 0u; instance_index < scene->instances.size(); instance_index++) {
        auto base_instance = scene->instances[instance_index];
        if (auto o = base_instance->object;
            o == minipbrt::kInvalidIndex || scene->objects[o]->firstShape == minipbrt::kInvalidIndex) {
            eprintln("Ignored instance at index {} with invalid object index.", instance_index);
        } else {
            if (base_instance->insideMedium != minipbrt::kInvalidIndex ||
                base_instance->outsideMedium != minipbrt::kInvalidIndex) {
                eprintln("Ignored unsupported instance inside medium at index {}.", instance_index);
            }
            if (base_instance->reverseOrientation) {
                eprintln("Ignored unsupported instance reverse orientation at index {}.", instance_index);
            }
            auto instance = nlohmann::json::object();
            instance["type"] = "Shape";
            instance["impl"] = "Instance";
            auto &prop = (instance["prop"] = nlohmann::json::object());
            // transform
            if (auto t = convert_transform(base_instance->instanceToWorld); !t.is_null()) {
                prop["transform"] = std::move(t);
            }
            if (auto l = base_instance->areaLight; l != minipbrt::kInvalidIndex) {
                prop["light"] = luisa::format("@AreaLight:{}", l);
            }
            prop["shape"] = luisa::format("@Object:{}", o);
            converted[luisa::format("Instance:{}", instance_index)] = instance;
            converted["render"]["shapes"].emplace_back(luisa::format("@Instance:{}", instance_index));
        }
    }
}

static void convert_area_lights(const minipbrt::Scene *scene,
                                nlohmann::json &converted) noexcept {
    for (auto i = 0u; i < scene->areaLights.size(); i++) {
        auto base_light = scene->areaLights[i];
        expect(base_light->type() == minipbrt::AreaLightType::Diffuse,
               "Unsupported area light at index {} with type {}.",
               i, magic_enum::enum_name(base_light->type()));
        auto diffuse = static_cast<const minipbrt::DiffuseAreaLight *>(base_light);
        auto light = nlohmann::json::object();
        light["type"] = "Light";
        light["impl"] = "Diffuse";
        auto &prop = (light["prop"] = nlohmann::json::object());
        prop["emission"] = {
            {"impl", "Constant"},
            {"prop",
             {{"v", nlohmann::json::array({base_light->scale[0] * diffuse->L[0],
                                           base_light->scale[1] * diffuse->L[1],
                                           base_light->scale[2] * diffuse->L[2]})}}}};
        prop["two_sided"] = diffuse->twosided;
        converted[luisa::format("AreaLight:{}", i)] = light;
    }
}

static void convert_textures(const std::filesystem::path &base_dir,
                             const minipbrt::Scene *scene,
                             nlohmann::json &converted) noexcept {
    for (auto texture_index = 0u; texture_index < scene->textures.size(); texture_index++) {
        auto base_texture = scene->textures[texture_index];
        nlohmann::json texture;
        texture["type"] = "Texture";
        auto &prop = (texture["prop"] = nlohmann::json::object());
        texture["impl"] = "Constant";
        prop["v"] = {1.f, 1.f, 1.f, 1.f};
        switch (base_texture->type()) {
            case minipbrt::TextureType::Scale: {
                auto s = static_cast<const minipbrt::ScaleTexture *>(base_texture);
                texture["impl"] = "Multiply";
                color_tex_parsing(scene, prop, "a", s->tex1);
                color_tex_parsing(scene, prop, "b", s->tex2);
                break;
            }
            case minipbrt::TextureType::Constant: {
                auto c = static_cast<const minipbrt::ConstantTexture *>(base_texture);
                texture["impl"] = "Constant";
                prop["v"] = {c->value[0], c->value[1], c->value[2]};
                break;
            }
            case minipbrt::TextureType::ImageMap: {
                auto image = static_cast<const minipbrt::ImageMapTexture *>(base_texture);
                expect(image->filename != nullptr, "Image filename is null.");
                try {
                    std::filesystem::path file{image->filename};
                    if (!file.is_absolute()) { file = base_dir / file; }
                    file = std::filesystem::canonical(file);
                    auto copied_file = luisa::format("lr_exported_textures/{:05}_{}",
                                                     texture_index, file.filename().generic_string());
                    std::filesystem::create_directories(base_dir / "lr_exported_textures");
                    std::filesystem::copy(file, base_dir / copied_file, std::filesystem::copy_options::update_existing);
                    texture["impl"] = "Image";
                    if (auto mapping = image->mapping; mapping == minipbrt::TexCoordMapping::UV) {
                        prop["uv_scale"] = {image->uscale, image->vscale};
                        prop["uv_offset"] = {image->udelta, image->vdelta};
                        switch (image->wrap) {
                            case minipbrt::WrapMode::Repeat: prop["address"] = "repeat"; break;
                            case minipbrt::WrapMode::Black: prop["address"] = "zero"; break;
                            case minipbrt::WrapMode::Clamp: prop["address"] = "edge"; break;
                        }
                    } else {
                        eprintln("Ignored unsupported texture mapping at index {} with type '{}'.",
                                 texture_index, magic_enum::enum_name(mapping));
                    }
                    prop["scale"] = image->scale;
                    prop["file"] = copied_file;
                    if (image->dataType == minipbrt::TextureData::Float) {
                        prop["encoding"] = "Linear";
                    } else if (image->gamma) {
                        prop["encoding"] = "sRGB";
                    }
                } catch (const std::exception &e) {
                    eprintln("Failed to copy image file: {}.", e.what());
                }
                break;
            }
            default: {
                eprintln("Ignored unsupported texture at index {} with "
                         "type '{}'. Falling back to constant.",
                         texture_index, magic_enum::enum_name(base_texture->type()));
                break;
            }
        }
        converted[texture_name(scene, texture_index)] = texture;
    }
}

[[nodiscard]] static std::string convert_bump_to_normal(const std::filesystem::path &base_dir,
                                                        const minipbrt::Scene *scene,
                                                        uint32_t bump_map_index,
                                                        nlohmann::json &converted) noexcept {
    auto base_texture_name = texture_name(scene, bump_map_index);
    auto base_texture = scene->textures[bump_map_index];
    if (base_texture->type() != minipbrt::TextureType::ImageMap) {
        eprintln("Ignored unsupported bump map at index {} with type '{}'.", bump_map_index,
                 magic_enum::enum_name(base_texture->type()));
        return {};
    }
    return {};
}

static void convert_materials(const std::filesystem::path &base_dir,
                              const minipbrt::Scene *scene,
                              nlohmann::json &converted) noexcept {
    for (auto i = 0u; i < scene->materials.size(); i++) {
        auto base_material = scene->materials[i];
        auto material = nlohmann::json::object();
        material["type"] = "Surface";
        material["impl"] = "Matte";
        auto &prop = (material["prop"] = nlohmann::json::object());
        prop["source"] = magic_enum::enum_name(base_material->type());
        if (auto b = base_material->bumpmap; b != minipbrt::kInvalidIndex) {
            if (auto normal_texture_name = convert_bump_to_normal(base_dir, scene, b, converted);
                !normal_texture_name.empty()) {
                prop["normal_map"] = luisa::format("@{}", normal_texture_name);
            } else {
                eprintln("Ignored unsupported bump map at index {}.", i);
            }
        }
        // TODO
        switch (auto material_type = base_material->type()) {
            case minipbrt::MaterialType::Disney: {
                auto disney_material = static_cast<minipbrt::DisneyMaterial *>(base_material);
                material["impl"] = "Disney";
                color_tex_parsing(scene, prop, "Kd", disney_material->color);
                float_tex_parsing(scene, prop, "anisotropic", disney_material->anisotropic);
                float_tex_parsing(scene, prop, "clearcoat", disney_material->clearcoat);
                float_tex_parsing(scene, prop, "clearcoat_gloss", disney_material->clearcoatgloss);
                float_tex_parsing(scene, prop, "eta", disney_material->eta);
                float_tex_parsing(scene, prop, "metallic", disney_material->metallic);
                float_tex_parsing(scene, prop, "roughness", disney_material->roughness);
                // TODO scatterdistance
                float_tex_parsing(scene, prop, "sheen", disney_material->sheen);
                float_tex_parsing(scene, prop, "sheen_tint", disney_material->sheentint);
                float_tex_parsing(scene, prop, "specular_trans", disney_material->spectrans);
                prop["thin"] = disney_material->thin;
                color_tex_parsing(scene, prop, "diffuse_trans", disney_material->difftrans);
                color_tex_parsing(scene, prop, "flatness", disney_material->flatness);
                break;
            }
            case minipbrt::MaterialType::Glass: {
                auto glass_material = static_cast<minipbrt::GlassMaterial *>(base_material);
                material["impl"] = "Glass";
                color_tex_parsing(scene, prop, "Kr", glass_material->Kr);
                color_tex_parsing(scene, prop, "Kt", glass_material->Kt);
                float_tex_parsing(scene, prop, "eta", glass_material->eta);
                std::vector<const minipbrt::FloatTex *> roughness{
                    &glass_material->uroughness,
                    &glass_material->vroughness};
                concat_tex_parsing(scene, prop, "roughness", roughness);
                prop["remap_roughness"] = glass_material->remaproughness;
                break;
            }
            case minipbrt::MaterialType::Matte: {
                auto matte_material = static_cast<minipbrt::MatteMaterial *>(base_material);
                material["impl"] = "Matte";
                color_tex_parsing(scene, prop, "Kd", matte_material->Kd);
                float_tex_parsing(scene, prop, "sigma", matte_material->sigma);
                break;
            }
            case minipbrt::MaterialType::Metal: {
                // TODO
                eprintln("Ignored unsupported material at index {} with type '{}'.",
                         i, magic_enum::enum_name(material_type));
                break;

                auto metal_material = static_cast<minipbrt::MetalMaterial *>(base_material);
                material["impl"] = "Metal";
                std::vector<const minipbrt::FloatTex *> roughness{
                    &metal_material->uroughness,
                    &metal_material->vroughness};
                concat_tex_parsing(scene, prop, "roughness", roughness);
                metal_eta_k_parsing(scene, prop, "eta", metal_material->eta, metal_material->k);
                prop["remap_roughness"] = metal_material->remaproughness;
                break;
            }
            case minipbrt::MaterialType::Mirror: {
                auto mirror_material = static_cast<minipbrt::MirrorMaterial *>(base_material);
                material["impl"] = "Mirror";
                color_tex_parsing(scene, prop, "Kr", mirror_material->Kr);
                break;
            }
            case minipbrt::MaterialType::Mix: {
                auto mix_material = static_cast<minipbrt::MixMaterial *>(base_material);
                material["impl"] = "Mix";
                prop["a"] = "@" + material_name(scene, mix_material->namedmaterial1);
                prop["b"] = "@" + material_name(scene, mix_material->namedmaterial2);
                color_tex_parsing(scene, prop, "ratio", mix_material->amount);
                break;
            }
            case minipbrt::MaterialType::None: break;
            case minipbrt::MaterialType::Plastic: {
                auto plastic_material = static_cast<minipbrt::PlasticMaterial *>(base_material);
                material["impl"] = "Plastic";
                color_tex_parsing(scene, prop, "Kd", plastic_material->Kd);
                float_tex_parsing(scene, prop, "roughness", plastic_material->roughness);
                prop["remap_roughness"] = plastic_material->remaproughness;
                break;
            }
            case minipbrt::MaterialType::Substrate: {
                auto substrate_material = static_cast<minipbrt::SubstrateMaterial *>(base_material);
                material["impl"] = "Plastic";
                color_tex_parsing(scene, prop, "Kd", substrate_material->Kd);
                std::vector<const minipbrt::FloatTex *> roughness{
                    &substrate_material->uroughness,
                    &substrate_material->vroughness};
                concat_tex_parsing(scene, prop, "roughness", roughness);
                prop["remap_roughness"] = substrate_material->remaproughness;
                break;
            }
            case minipbrt::MaterialType::Translucent: {
                auto translucent_material = static_cast<minipbrt::TranslucentMaterial *>(base_material);
                material["impl"] = "Disney";
                color_tex_parsing(scene, prop, "Kd", translucent_material->Kd);
                color_tex_parsing(scene, prop, "specular_trans", translucent_material->Ks);
                float_tex_parsing(scene, prop, "roughness", translucent_material->roughness);
                prop["thin"] = true;
                prop["remap_roughness"] = translucent_material->remaproughness;
                break;
            }
            case minipbrt::MaterialType::Uber: {
                auto uber_material = static_cast<minipbrt::UberMaterial *>(base_material);
                material["impl"] = "Disney";
                color_tex_parsing(scene, prop, "Kd", uber_material->Kd);
                float_tex_parsing(scene, prop, "eta", uber_material->eta);
                //                color_tex_parsing(scene, prop, "metallic", uber_material->Ks);
                std::vector<const minipbrt::FloatTex *> roughness{
                    &uber_material->uroughness,
                    &uber_material->vroughness};
                concat_tex_parsing(scene, prop, "roughness", roughness);
                color_tex_parsing(scene, prop, "alpha", uber_material->opacity);
                color_tex_parsing(scene, prop, "specular_trans", uber_material->Kt);
                prop["remap_roughness"] = uber_material->remaproughness;
                break;
            }
                //            case minipbrt::MaterialType::Subsurface:
                //            case minipbrt::MaterialType::Fourier:
                //            case minipbrt::MaterialType::Hair:
                //            case minipbrt::MaterialType::KdSubsurface: {
                //                // TODO
                //                eprintln("Ignored unsupported material at index {} with type '{}'.",
                //                         i, magic_enum::enum_name(material_type));
                //                break;
                //            }
            default: eprintln("Ignored unsupported material at index {} with type '{}'.",
                              i, magic_enum::enum_name(material_type));
        }
        auto name = material_name(scene, i);
        converted[name] = material;
    }
}

[[nodiscard]] static nlohmann::json convert_film(const minipbrt::Film *base_film) noexcept {
    expect(base_film->type() == minipbrt::FilmType::Image,
           "Unsupported film type {}.", magic_enum::enum_name(base_film->type()));
    auto image = static_cast<const minipbrt::ImageFilm *>(base_film);
    auto film = nlohmann::json::object();
    auto max_lum = image->maxsampleluminance <= 0.f ? 65536.f : image->maxsampleluminance;
    return {
        {"impl", "Color"},
        {"prop",
         {{"resolution", nlohmann::json::array({image->xresolution, image->yresolution})},
          {"exposure", std::log2(image->scale)},
          {"clamp", std::clamp(max_lum, 16.f, 65536.f)}}}};
}

[[nodiscard]] static nlohmann::json convert_filter(const minipbrt::Filter *base_filter) noexcept {
    auto radius = std::max((base_filter->xwidth + base_filter->ywidth) / 2., 1.);
    return {{"impl", "Gaussian"},
            {"prop", {{"radius", radius}}}};
}

static void convert_camera(const minipbrt::Scene *scene,
                           nlohmann::json &converted) noexcept {
    auto base_camera = scene->camera;
    expect(base_camera->type() == minipbrt::CameraType::Perspective,
           "Unsupported camera type {}.", magic_enum::enum_name(base_camera->type()));
    auto camera = nlohmann::json::object();
    camera["type"] = "Camera";
    auto perspective = static_cast<const minipbrt::PerspectiveCamera *>(base_camera);
    auto &prop = (camera["prop"] = nlohmann::json::object());
    auto w = 0, h = 0;
    scene->film->get_resolution(w, h);
    auto a = static_cast<double>(w) / static_cast<double>(h);
    if (perspective->lensradius > 0.f) {
        auto uncropped = 12 / (a < 1. ? 1.5 * a : a / 1.5);
        auto focal_length = uncropped / std::tan(radians(perspective->fov / 2.));
        auto focus_distance = perspective->focaldistance;
        auto lens_radius = perspective->lensradius * 1000.;
        camera["impl"] = "ThinLens";
        prop["focal_length"] = focal_length;
        prop["focus_distance"] = focus_distance;
        prop["aperture"] = focal_length / (2. * lens_radius);
    } else {
        camera["impl"] = "Pinhole";
        prop["fov"] = [f = static_cast<double>(perspective->fov), a] {
            if (a < 1.) {// convert from horizontal to vertical
                auto half_w = std::tan(radians(f) / 2.);
                auto half_h = half_w / a;
                return 2. * degrees(std::atan(half_h));
            }
            return f;
        }();
    }
    prop["transform"] = convert_camera_transform(perspective->cameraToWorld);
    prop["film"] = convert_film(scene->film);
    if (auto filter = scene->filter) { prop["filter"] = convert_filter(filter); }
    prop["file"] = [scene]() noexcept -> std::string {
        if (auto film = dynamic_cast<const minipbrt::ImageFilm *>(scene->film);
            film != nullptr && film->filename != nullptr) {
            std::filesystem::path name{film->filename};
            name.replace_extension(".exr");
            return name.generic_string();
        }
        return "render.exr";
    }();
    prop["spp"] = 64;
    converted["render"]["cameras"] = nlohmann::json::array({camera});
}

static void dump_converted_scene(const std::filesystem::path &base_dir,
                                 std::string_view name,
                                 nlohmann::json converted) noexcept {
    auto render = std::move(converted["render"]);
    converted.erase("render");
    auto shapes = std::move(render["shapes"]);
    render.erase("shapes");
    converted["renderable"] = {
        {"type", "Shape"},
        {"impl", "Group"},
        {"prop", {{"shapes", std::move(shapes)}}}};
    render["shapes"] = nlohmann::json::array({"@renderable"});
    nlohmann::json entry = {
        {"render", std::move(render)},
        {"import", nlohmann::json::array({luisa::format("{}.exported.json", name)})},
    };
    auto write_json = [&base_dir](std::string_view file_name, const nlohmann::json &json) {
        std::ofstream f{base_dir / file_name};
        f << json.dump(4);
    };
    write_json(luisa::format("{}.exported.json", name), converted);
    write_json(luisa::format("{}.json", name), entry);
}

static void convert_lights(const std::filesystem::path &base_dir,
                           const minipbrt::Scene *scene,
                           nlohmann::json &converted) {
    std::vector<std::string> env_array;
    for (auto light_index = 0u; light_index < scene->lights.size(); light_index++) {
        auto base_light = scene->lights[light_index];
        if (base_light->scale[0] <= 0.f && base_light->scale[1] <= 0.f && base_light->scale[2] <= 0.f) {
            eprintln("Ignored light at index {} with invalid scale.", light_index);
            continue;
        }
        auto light = nlohmann::json::object({{"type", "Light"}, {"impl", "Diffuse"}, {"prop", nlohmann::json::object()}});
        auto &emission = light["emission"];
        emission["type"] = "Texture";
        emission["impl"] = "Constant";
        switch (auto light_type = base_light->type()) {
            case minipbrt::LightType::Point: {
                // light
                auto point_light = static_cast<minipbrt::PointLight *>(base_light);
                emission["prop"] = nlohmann::json::object({{"v", nlohmann::json::array({
                                                                     base_light->scale[0] * point_light->I[0],
                                                                     base_light->scale[1] * point_light->I[1],
                                                                     base_light->scale[2] * point_light->I[2],
                                                                 })}});

                // shape
                auto light_shape = nlohmann::json::object(
                    {{"type", "Shape"},
                     {"impl", "Sphere"},
                     {"prop", {{"visible", false}}}});
                auto &prop = (light_shape["prop"] = nlohmann::json::object());
                // transform
                auto position_transform = nlohmann::json::object(
                    {{"type", "transform"},
                     {"impl", "SRT"},
                     {"prop",
                      {{"translate",
                        nlohmann::json::array({point_light->from[0],
                                               point_light->from[1],
                                               point_light->from[2]})}}}});

                prop["transform"] = nlohmann::json::object({{"type", "transform"}, {"impl", "Stack"}});
                prop["transform"]["prop"] = nlohmann::json::object(
                    {{"transforms",
                      nlohmann::json::array({position_transform, convert_transform(base_light->lightToWorld)})}});
                prop["light"] = light;
                //                converted[luisa::format("Light:{}", light_index)] = light;
                break;
            }
            case minipbrt::LightType::Distant: {
                auto distant_light = static_cast<minipbrt::DistantLight *>(base_light);
                if (auto L = distant_light->L; !(L[0] <= 0.f && L[1] <= 0.f && L[2] <= 0.f)) {
                    auto env = nlohmann::json::object(
                        {{"type", "Environment"},
                         {"impl", "Directional"}});
                    auto &prop = (env["prop"] = nlohmann::json::object());
                    prop["visible"] = false;
                    prop["normalize"] = true;
                    auto s = std::numbers::inv_pi / 4.;
                    auto &e = (prop["emission"] = nlohmann::json::object(
                                   {{"type", "Texture"},
                                    {"impl", "Constant"},
                                    {"prop", {{"v", nlohmann::json::array({L[0] * s, L[1] * s, L[2] * s})}}}}));
                    glm::mat4 m;
                    for (auto i = 0; i < 4; i++) {
                        for (auto j = 0; j < 4; j++) {
                            m[i][j] = distant_light->lightToWorld.start[j][i];
                        }
                    }
                    println("Distant light matrix: (({}, {}, {}, {}), ({}, {}, {}, {}), ({}, {}, {}, {}), ({}, {}, {}, {})).",
                            m[0][0], m[0][1], m[0][2], m[0][3],
                            m[1][0], m[1][1], m[1][2], m[1][3],
                            m[2][0], m[2][1], m[2][2], m[2][3],
                            m[3][0], m[3][1], m[3][2], m[3][3]);
                    auto d = glm::vec3(m * glm::vec4(distant_light->to[0], distant_light->to[1], distant_light->to[2], 1.f)) -
                             glm::vec3(m * glm::vec4(distant_light->from[0], distant_light->from[1], distant_light->from[2], 1.f));
                    auto dd =//glm::mat3(glm::rotate(glm::mat4(1.f), -.5f * std::numbers::pi_v<float>, glm::vec3(0, 0, 1))) *
                        //glm::mat3(glm::scale(glm::mat4(1.f), glm::vec3(1, -1, 1))) *
                        //glm::mat3(glm::rotate(glm::mat4(1.f), .5f * std::numbers::pi_v<float>, glm::vec3(1, 0, 0))) *
                        glm::normalize(d);
                    prop["direction"] = nlohmann::json::array({-dd.x, -dd.y, dd.z});
                    luisa::println("Directional light direction: ({}, {}, {}).", -dd.x, -dd.y, dd.z);
                    if (auto scale = base_light->scale;
                        scale[0] == scale[1] && scale[1] == scale[2]) {
                        prop["scale"] = scale[0];
                    } else {
                        auto base_emission = std::move(e);
                        prop["emission"] = {
                            {"impl", "Scale"},
                            {"prop", {{"base", std::move(base_emission)}, {"scale", {scale[0], scale[1], scale[2]}}}}};
                    }
                    auto name = luisa::format("Env:{}:Directional", light_index);
                    converted[name] = env;
                    env_array.emplace_back("@" + name);
                }
                break;
            }
            case minipbrt::LightType::Infinite: {
                auto infinite_light = static_cast<minipbrt::InfiniteLight *>(base_light);
                auto env = nlohmann::json::object(
                    {{"type", "Environment"},
                     {"impl", "Spherical"}});
                auto &prop = (env["prop"] = nlohmann::json::object());
                auto &e = (prop["emission"] = nlohmann::json::object());
                if (auto map = infinite_light->mapname) {
                    auto file = [&base_dir, map] {
                        try {
                            std::filesystem::path file{map};
                            if (!file.is_absolute()) { file = base_dir / file; }
                            return std::filesystem::canonical(file);
                        } catch (const std::exception &ex) {
                            panic("Failed to resolve image file path: {}.", ex.what());
                        }
                    }();
                    auto copied_file = luisa::format("lr_exported_textures/env_{:05}_{}",
                                                     light_index, file.filename().generic_string());
                    try {
                        std::filesystem::create_directories(base_dir / "lr_exported_textures");
                        std::filesystem::copy(file, base_dir / copied_file, std::filesystem::copy_options::update_existing);
                    } catch (const std::exception &ex) {
                        panic("Failed to copy image file: {}.", ex.what());
                    }
                    e["impl"] = "Image";
                    e["prop"] = {{"file", copied_file}};
                } else {
                    e["impl"] = "Constant";
                    e["prop"] = {{"v",
                                  nlohmann::json::array({infinite_light->L[0],
                                                         infinite_light->L[1],
                                                         infinite_light->L[2]})}};
                }
                if (auto scale = base_light->scale;
                    scale[0] == scale[1] && scale[1] == scale[2]) {
                    prop["scale"] = scale[0];
                } else {
                    auto base_emission = std::move(e);
                    prop["emission"] = {
                        {"impl", "Scale"},
                        {"prop", {{"base", std::move(base_emission)}, {"scale", {scale[0], scale[1], scale[2]}}}}};
                }
                auto name = luisa::format("Env:{}:Spherical", light_index);
                prop["transform"] = convert_envmap_transform(base_light->lightToWorld);
                converted[name] = env;
                env_array.emplace_back("@" + name);
                break;
            }
            default: eprintln("Ignored unsupported light at index {} with type '{}'.",
                              light_index, magic_enum::enum_name(light_type));
        }
    }
    println("Environment count: {}", env_array.size());
    if (env_array.size() == 1u) {
        converted["render"]["environment"] = env_array[0];
    } else if (env_array.size() > 1u) {
        converted["render"]["environment"] = nlohmann::json::object({{"type", "Environment"}, {"impl", "Grouped"}, {"prop", {{"environments", nlohmann::json::array()}}}});
        for (auto &item : env_array) {
            converted["render"]["environment"]["prop"]["environments"].emplace_back(item);
        }
    }
}

static void convert_scene(const std::filesystem::path &source_path,
                          const minipbrt::Scene *scene) noexcept {
    try {
        println("Time: {} -> {}", scene->startTime, scene->endTime);
        println("Medium count: {}", scene->mediums.size());
        auto base_dir = source_path.parent_path();
        nlohmann::json converted{
            {"render",
             {{"integrator",
               {{"impl", "MegaPath"},
                {"prop",
                 {{"depth", 16},
                  {"rr_depth", 5},
                  {"sampler", {{"impl", "ZSobol"}}}}}}},
              {"shapes", nlohmann::json::array()}}}};
        convert_textures(base_dir, scene, converted);
        convert_materials(base_dir, scene, converted);
        convert_area_lights(scene, converted);
        convert_shapes(base_dir, scene, converted);
        convert_lights(base_dir, scene, converted);
        convert_camera(scene, converted);
        auto name = source_path.stem().generic_string();
        dump_converted_scene(base_dir, name, std::move(converted));
    } catch (const std::exception &e) {
        luisa::panic("{}", e.what());
    }
}

void convert(const char *scene_file_name) noexcept {
    try {
        auto scene_file = std::filesystem::canonical(scene_file_name);
        minipbrt::Loader loader;
        if (loader.load(scene_file.generic_string().c_str())) {
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
            auto message = e ? luisa::format("{} [{}:{}:{}]",
                                             e->message(), e->filename(), e->line(), e->column()) :
                               "unknown";
            luisa::panic("Failed to load scene file {}: {}", scene_file.generic_string(), message);
        }
    } catch (const std::exception &e) {
        luisa::panic("{}", e.what());
    }
}

}// namespace luisa::render