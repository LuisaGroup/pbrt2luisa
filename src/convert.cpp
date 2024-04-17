//
// Created by Mike on 2024/4/16.
//

#include <fstream>
#include <filesystem>

#include <nlohmann/json.hpp>
#include <magic_enum/magic_enum.hpp>
#include <glm/glm.hpp>
#include <minipbrt.h>

#include "logging.h"
#include "convert.h"

namespace luisa::render {

[[nodiscard]] static nlohmann::json convert_transform(
    const minipbrt::Scene *scene,
    const minipbrt::Transform &transform) noexcept {
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

[[nodiscard]] static nlohmann::json convert_camera_transform(
    const minipbrt::Scene *scene,
    const minipbrt::Transform &transform) noexcept {
    /*
    def transform(v):
        v = np.reshape(np.array(v + [1.0]), [4, 1])
        v = np.reshape(np.matmul(matrix, v), [4])
        return v[:3] / v[3]

    def transform_normal(n):
        n = np.reshape(np.array(n), [3, 1])
        n = np.reshape(np.matmul(np.transpose(np.linalg.inv(matrix[:3, :3])), n), [3])
        n = np.array([e if abs(e) > 1e-5 else 0.0 for e in n])
        return n / np.linalg.norm(n)

    eye = transform([0.0, 0.0, 0.0])
    up = transform_normal([0.0, 1.0, 0.0])
    lookat = eye + transform_normal([0.0, 0.0, 1.0])
    scene["Camera"].append([["time", [0.0]], ["position", eye], ["up", up], ["lookAt", lookat]])
    return scene, loc
     */
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
    return {
        {"impl", "View"},
        {"prop",
         {{"origin", {eye.x, eye.y, eye.z}},
          {"front", {front.x, front.y, front.z}},
          {"up", {up.x, up.y, up.z}}}}};
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
        if (auto t = convert_transform(scene, base_shape->shapeToWorld); !t.is_null()) {
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
        if (auto t = convert_transform(scene, base_object->objectToInstance); !t.is_null()) {
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
            if (auto t = convert_transform(scene, base_instance->instanceToWorld); !t.is_null()) {
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
             {"v",
              {base_light->scale[0] * diffuse->L[0],
               base_light->scale[1] * diffuse->L[1],
               base_light->scale[2] * diffuse->L[2]}}}};
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
            case minipbrt::TextureType::Constant: {
                auto c = static_cast<const minipbrt::ConstantTexture *>(base_texture);
                texture["impl"] = "Constant";
                prop["v"] = {c->value[0], c->value[1], c->value[2]};
                break;
            }
            case minipbrt::TextureType::ImageMap: {
                auto image = static_cast<const minipbrt::ImageMapTexture *>(base_texture);
                texture["impl"] = "Image";
                if (auto mapping = image->mapping; mapping == minipbrt::TexCoordMapping::UV) {
                    prop["uv_scale"] = {image->uscale, image->vscale};
                    prop["uv_offset"] = {image->udelta, image->vdelta};
                } else {
                    eprintln("Ignored unsupported texture mapping at index {} with type '{}'.",
                             texture_index, magic_enum::enum_name(mapping));
                }
                prop["scale"] = image->scale;
                expect(image->filename != nullptr, "Image filename is null.");
                auto file = [&base_dir, image] {
                    try {
                        std::filesystem::path file{image->filename};
                        if (!file.is_absolute()) { file = base_dir / file; }
                        return std::filesystem::canonical(file);
                    } catch (const std::exception &e) {
                        panic("Failed to resolve image file path: {}.", e.what());
                    }
                }();
                auto copied_file = luisa::format("lr_exported_textures/{:05}_{}", texture_index, file.filename().string());
                try {
                    std::filesystem::create_directories(base_dir / "lr_exported_textures");
                    std::filesystem::copy(file, base_dir / copied_file, std::filesystem::copy_options::update_existing);
                } catch (const std::exception &e) {
                    panic("Failed to copy image file: {}.", e.what());
                }
                prop["file"] = copied_file;
                if (image->gamma) { prop["encoding"] = "sRGB"; }
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

static void color_tex_parsing(
    const minipbrt::Scene *scene,
    nlohmann::json &prop,
    const std::string &name,
    const minipbrt::ColorTex &tex) {
    if (tex.texture == minipbrt::kInvalidIndex) {
        prop[name] = nlohmann::json::object({{"type", "Texture"}, {"impl", "Constant"}, {"prop", {{"v", {
                                                                                                            tex.value[0],
                                                                                                            tex.value[1],
                                                                                                            tex.value[2],
                                                                                                        }}}}});
    } else {
        prop[name] = "@" + texture_name(scene, tex.texture);
    }
}

static void float_tex_parsing(
    const minipbrt::Scene *scene,
    nlohmann::json &prop,
    const std::string &name,
    const minipbrt::FloatTex &tex) {
    if (tex.texture == minipbrt::kInvalidIndex) {
        prop[name] = nlohmann::json::object({{"type", "Texture"}, {"impl", "Constant"}, {"prop", {{"v", tex.value}}}});
    } else {
        prop[name] = "@" + texture_name(scene, tex.texture);
    }
}

static void convert_materials(const minipbrt::Scene *scene,
                              nlohmann::json &converted) noexcept {
    for (auto i = 0u; i < scene->materials.size(); i++) {
        auto base_material = scene->materials[i];
        if (base_material->bumpmap != minipbrt::kInvalidIndex) {
            eprintln("Ignored unsupported material bump map at index {}.", i);
        }
        auto material = nlohmann::json::object();
        material["type"] = "Surface";
        auto &prop = (material["prop"] = nlohmann::json::object());
        material["impl"] = "Matte";
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
                //            case minipbrt::MaterialType::Fourier: break;
            case minipbrt::MaterialType::Glass: {
                auto glass_material = static_cast<minipbrt::GlassMaterial *>(base_material);
                material["impl"] = "Glass";
                color_tex_parsing(scene, prop, "roughness", glass_material->Kr);
                color_tex_parsing(scene, prop, "Kt", glass_material->Kt);
                float_tex_parsing(scene, prop, "eta", glass_material->eta);
                break;
            }
                //            case minipbrt::MaterialType::Hair: break;
                //            case minipbrt::MaterialType::KdSubsurface: break;
            case minipbrt::MaterialType::Matte: {
                auto matte_material = static_cast<minipbrt::MatteMaterial *>(base_material);
                material["impl"] = "Matte";
                color_tex_parsing(scene, prop, "Kd", matte_material->Kd);
                float_tex_parsing(scene, prop, "sigma", matte_material->sigma);
                break;
            }
//            case minipbrt::MaterialType::Metal: {
//                //                auto metal_material = static_cast<minipbrt::MetalMaterial*>(base_material);
//                //                material["impl"] = "Metal";
//                //                color_tex_parsing(scene, prop, "Kr", metal_material->Kr);
//                //TODO check
//                break;
//            }
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
                float_tex_parsing(scene, prop, "roughness", substrate_material->uroughness);// TODO: uv roughness
                prop["remap_roughness"] = substrate_material->remaproughness;
                break;
            }
                //            case minipbrt::MaterialType::Subsurface: break;
            case minipbrt::MaterialType::Translucent: {
                auto translucent_material = static_cast<minipbrt::TranslucentMaterial *>(base_material);
                material["impl"] = "Plastic";
                color_tex_parsing(scene, prop, "Kd", translucent_material->Kd);
                float_tex_parsing(scene, prop, "roughness", translucent_material->roughness);
                break;
            }
            case minipbrt::MaterialType::Uber: {
                auto uber_material = static_cast<minipbrt::UberMaterial *>(base_material);
                material["impl"] = "Disney";
                color_tex_parsing(scene, prop, "Kd", uber_material->Kd);
                float_tex_parsing(scene, prop, "eta", uber_material->eta);
                color_tex_parsing(scene, prop, "metallic", uber_material->Ks);
                float_tex_parsing(scene, prop, "roughness", uber_material->uroughness);
                color_tex_parsing(scene, prop, "alpha", uber_material->opacity);
                color_tex_parsing(scene, prop, "specular_trans", uber_material->Kt);
                break;
            }
            default: eprintln("Ignored unsupported material at index {} with type '{}'.",
                              i, magic_enum::enum_name(material_type));
        }
        auto name = material_name(scene, i);
        converted[name] = material;
    }
}

[[nodiscard]] static nlohmann::json convert_film(const minipbrt::Scene *scene,
                                                 nlohmann::json &converted) noexcept {
    auto base_film = scene->film;
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

static void convert_camera(const minipbrt::Scene *scene,
                           nlohmann::json &converted) noexcept {
    auto base_camera = scene->camera;
    expect(base_camera->type() == minipbrt::CameraType::Perspective,
           "Unsupported camera type {}.", magic_enum::enum_name(base_camera->type()));
    auto camera = nlohmann::json::object();
    camera["type"] = "Camera";
    auto perspective = static_cast<const minipbrt::PerspectiveCamera *>(base_camera);
    auto &prop = (camera["prop"] = nlohmann::json::object());
    if (perspective->lensradius > 0.f) {
        // FOV = 2 arctan h/2f => fov / 2 = arctan h / 2f => tan(fov / 2) = h / 2f =>
        auto focal_length = (24. / 2.) / (std::tan(glm::radians(perspective->fov / 2.)));
        auto focus_distance = perspective->focaldistance;
        camera["impl"] = "ThinLens";
        prop["focal_length"] = focal_length;
        prop["focus_distance"] = focus_distance;
    } else {
        camera["impl"] = "Pinhole";
    }
    prop["transform"] = convert_camera_transform(scene, perspective->cameraToWorld);
    prop["fov"] = perspective->fov;
    prop["film"] = convert_film(scene, converted);
    prop["file"] = "render.exr";
    prop["spp"] = 1024;
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
    for (auto light_index = 0u; light_index < scene->lights.size(); light_index++) {
        auto base_light = scene->lights[light_index];
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
                auto light_shape = nlohmann::json::object({{"type", "Shape"}, {"impl", "Sphere"}});
                auto &prop = (light_shape["prop"] = nlohmann::json::object());
                // transform
                auto position_transform = nlohmann::json::object({{"type", "transform"}, {"impl", "SRT"}, {"prop", nlohmann::json::object({{"translate", nlohmann::json::array({point_light->from[0], point_light->from[1], point_light->from[2]})}})}});

                prop["transform"] = nlohmann::json::object({{"type", "transform"}, {"impl", "Stack"}});
                prop["transform"]["prop"] = nlohmann::json::object({{"transforms", nlohmann::json::array({position_transform,
                                                                                                          convert_transform(scene, base_light->lightToWorld)})}});
                prop["light"] = light;
                //                converted[luisa::format("Light:{}", light_index)] = light;
                break;
            }
            case minipbrt::LightType::Infinite: {
                auto infinite_light = static_cast<minipbrt::InfiniteLight *>(base_light);
                auto env = nlohmann::json::object(
                    {{"type", "Environment"},
                     {"impl", "Spherical"},
                     {"prop",
                      {{"scale",
                        {infinite_light->scale[0],
                         infinite_light->scale[1],
                         infinite_light->scale[2]}}}}});
                auto &e = (env["prop"]["emission"] = nlohmann::json::object());
                if (auto map = infinite_light->mapname) {
                    auto file = [&base_dir, map] {
                        try {
                            std::filesystem::path file{map};
                            if (!file.is_absolute()) { file = base_dir / file; }
                            return std::filesystem::canonical(file);
                        } catch (const std::exception &e) {
                            panic("Failed to resolve image file path: {}.", e.what());
                        }
                    }();
                    auto copied_file = luisa::format("lr_exported_textures/env_{:05}_{}", light_index, file.filename().string());
                    try {
                        std::filesystem::create_directories(base_dir / "lr_exported_textures");
                        std::filesystem::copy(file, base_dir / copied_file, std::filesystem::copy_options::update_existing);
                    } catch (const std::exception &e) {
                        panic("Failed to copy image file: {}.", e.what());
                    }
                    e["impl"] = "Image";
                    e["prop"] = {{"file", copied_file}};
                } else {
                    e["impl"] = "Constant";
                    e["prop"] = {{"v", {infinite_light->L[0], infinite_light->L[1], infinite_light->L[2]}}};
                }
                auto name = luisa::format("EnvLight:{}", light_index);
                converted[name] = env;
                // TODO: use combine if existing
                converted["render"]["environment"] = "@" + name;
                break;
            }
            default: eprintln("Ignored unsupported light at index {} with type '{}'.",
                              light_index, magic_enum::enum_name(light_type));
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
                  {"rr_depth", 5}}}}},
              {"shapes", nlohmann::json::array()}}}};
        convert_textures(base_dir, scene, converted);
        convert_materials(scene, converted);
        convert_area_lights(scene, converted);
        convert_shapes(base_dir, scene, converted);
        convert_lights(base_dir, scene, converted);
        convert_camera(scene, converted);
        auto name = source_path.stem().string();
        dump_converted_scene(base_dir, name, std::move(converted));
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
            auto message = e ? luisa::format("{} [{}:{}:{}]",
                                             e->message(), e->filename(), e->line(), e->column()) :
                               "unknown";
            luisa::panic("Failed to load scene file {}: {}", scene_file.string(), message);
        }
    } catch (const std::exception &e) {
        luisa::panic("{}", e.what());
    }
}

}// namespace luisa::render