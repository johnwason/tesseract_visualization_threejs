#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <json/json.h>
#include <openssl/evp.h>

#include <tesseract/collision/types.h>
#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/visualization/threejs/viewer_state.h>
#include <tesseract/visualization/markers/arrow_marker.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/markers/contact_results_marker.h>
#include <tesseract/visualization/markers/geometry_marker.h>
#include <tesseract/visualization/markers/toolpath_marker.h>

using namespace tesseract::visualization::threejs;
using tesseract::common::GeneralResourceLocator;
using tesseract::common::JointId;
using tesseract::common::JointState;
using tesseract::common::JointTrajectory;
using tesseract::common::LinkId;
using tesseract::environment::Environment;

namespace
{
constexpr char SHAPES_URDF[] = R"(
<robot name="shapes" tesseract:make_convex="true" xmlns:tesseract="http://tesseract">
  <link name="world"/>
  <link name="cylinder_link">
    <visual><geometry><cylinder length="0.6" radius="0.2"/></geometry>
      <material name="red"><color rgba="0.8 0 0 1"/></material></visual>
  </link>
  <joint name="cylinder_joint" type="revolute">
    <parent link="world"/><child link="cylinder_link"/><axis xyz="0 1 0"/>
    <limit effort="0" lower="-2" upper="2" velocity="6"/>
  </joint>
  <link name="box_link">
    <visual><geometry><box size="0.6 0.1 0.2"/></geometry>
      <material name="green"><color rgba="0 0.8 0 1"/></material></visual>
  </link>
  <joint name="box_joint" type="prismatic">
    <parent link="cylinder_link"/><child link="box_link"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit effort="0" lower="-2" upper="2" velocity="6"/>
  </joint>
  <link name="box2_link">
    <visual><geometry><box size="1 2 3"/></geometry></visual>
  </link>
  <joint name="box2_joint" type="fixed">
    <parent link="world"/><child link="box2_link"/><origin xyz="0 2 0"/>
  </joint>
  <link name="sphere_link">
    <visual><geometry><sphere radius="0.5"/></geometry></visual>
  </link>
  <joint name="sphere_joint" type="fixed">
    <parent link="world"/><child link="sphere_link"/><origin xyz="0 3 0"/>
  </joint>
</robot>)";

Json::Value parse(std::string_view text)
{
  Json::CharReaderBuilder builder;
  const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value out;
  std::string errors;
  EXPECT_TRUE(reader->parse(text.data(), text.data() + text.size(), &out, &errors)) << errors;
  return out;
}

std::shared_ptr<Environment> shapesEnv()
{
  auto env = std::make_shared<Environment>();
  EXPECT_TRUE(env->init(std::string(SHAPES_URDF), std::make_shared<GeneralResourceLocator>()));
  return env;
}

std::string readResource(const std::string& url)
{
  GeneralResourceLocator locator;
  auto resource = locator.locateResource(url);
  if (resource == nullptr)
    throw std::runtime_error("Set TESSERACT_RESOURCE_PATH so this resolves: " + url);
  const auto bytes = resource->getResourceContents();
  return { bytes.begin(), bytes.end() };
}

std::shared_ptr<Environment> abbEnv()
{
  auto env = std::make_shared<Environment>();
  EXPECT_TRUE(env->init(readResource("package://tesseract/support/urdf/abb_irb2400.urdf"),
                        readResource("package://tesseract/support/urdf/abb_irb2400.srdf"),
                        std::make_shared<GeneralResourceLocator>()));
  return env;
}

struct Glb
{
  Json::Value json;
  std::vector<std::uint8_t> bin;
};

std::uint32_t readU32(const std::vector<std::uint8_t>& data, std::size_t offset)
{
  std::uint32_t v = 0;
  std::memcpy(&v, data.data() + offset, 4);
  return v;
}

Glb decodeGlb(const std::vector<std::uint8_t>& data)
{
  EXPECT_EQ(readU32(data, 0), 0x46546C67u);
  EXPECT_EQ(readU32(data, 4), 2u);
  EXPECT_EQ(readU32(data, 8), data.size());
  const std::uint32_t json_size = readU32(data, 12);
  EXPECT_EQ(readU32(data, 16), 0x4E4F534Au);
  EXPECT_EQ(json_size % 4, 0u);
  const std::size_t bin_header = 20 + json_size;
  const std::uint32_t bin_size = readU32(data, bin_header);
  EXPECT_EQ(readU32(data, bin_header + 4), 0x004E4942u);
  EXPECT_EQ(bin_size % 4, 0u);
  EXPECT_EQ(bin_header + 8 + bin_size, data.size());
  Glb out;
  out.json = parse(std::string_view(reinterpret_cast<const char*>(data.data()) + 20, json_size));
  out.bin.assign(data.begin() + static_cast<std::ptrdiff_t>(bin_header + 8), data.end());
  return out;
}

std::vector<std::uint8_t> base64Decode(const std::string& text)
{
  std::vector<std::uint8_t> out(text.size() / 4 * 3 + 1);
  const int written = EVP_DecodeBlock(out.data(), reinterpret_cast<const unsigned char*>(text.data()), static_cast<int>(text.size()));
  EXPECT_GE(written, 0);
  // EVP_DecodeBlock counts the bytes that the padding stands for
  const auto padding = static_cast<int>(std::count(text.end() - std::min<std::ptrdiff_t>(2, static_cast<std::ptrdiff_t>(text.size())), text.end(), '='));
  out.resize(static_cast<std::size_t>(written - padding));
  return out;
}

const Json::Value* findNode(const Json::Value& gltf, const std::string& name)
{
  for (const auto& node : gltf["nodes"])
    if (node["name"].asString() == name)
      return &node;
  ADD_FAILURE() << "No node named " << name;
  return nullptr;
}

template <class T>
std::vector<T> readAccessor(const Json::Value& gltf, const std::vector<std::uint8_t>& bin, int accessor_index)
{
  const Json::Value& accessor = gltf["accessors"][accessor_index];
  const Json::Value& view = gltf["bufferViews"][accessor["bufferView"].asInt()];
  const std::size_t offset = view["byteOffset"].asUInt64();
  const std::size_t length = view["byteLength"].asUInt64();
  EXPECT_LE(offset + length, bin.size());
  std::vector<T> out(length / sizeof(T));
  std::memcpy(out.data(), bin.data() + offset, length);
  return out;
}

/** @brief Every accessor must lie inside the buffer, and its bounds must match the decoded data */
void checkAccessors(const Glb& glb)
{
  static const std::map<std::string, std::size_t> COMPONENTS = { { "SCALAR", 1 }, { "VEC2", 2 }, { "VEC3", 3 }, { "VEC4", 4 } };
  ASSERT_TRUE(glb.json.isMember("accessors"));
  for (Json::ArrayIndex i = 0; i < glb.json["accessors"].size(); ++i)
  {
    const Json::Value& accessor = glb.json["accessors"][i];
    const std::size_t components = COMPONENTS.at(accessor["type"].asString());
    // Both component types in use are four bytes, and glTF requires alignment to the component size
    EXPECT_EQ(glb.json["bufferViews"][accessor["bufferView"].asInt()]["byteOffset"].asUInt64() % 4, 0u);
    ASSERT_EQ(accessor["min"].size(), components);
    ASSERT_EQ(accessor["max"].size(), components);

    const auto check = [&](const auto& values) {
      ASSERT_EQ(values.size(), accessor["count"].asUInt64() * components);
      for (std::size_t c = 0; c < components; ++c)
      {
        double min = std::numeric_limits<double>::max();
        double max = std::numeric_limits<double>::lowest();
        for (std::size_t k = c; k < values.size(); k += components)
        {
          min = std::min(min, static_cast<double>(values[k]));
          max = std::max(max, static_cast<double>(values[k]));
        }
        EXPECT_EQ(accessor["min"][static_cast<int>(c)].asDouble(), min);
        EXPECT_EQ(accessor["max"][static_cast<int>(c)].asDouble(), max);
      }
    };
    if (accessor["componentType"].asInt() == 5126)
      check(readAccessor<float>(glb.json, glb.bin, static_cast<int>(i)));
    else
    {
      ASSERT_EQ(accessor["componentType"].asInt(), 5125);
      check(readAccessor<std::uint32_t>(glb.json, glb.bin, static_cast<int>(i)));
    }
  }
}

void expectVec(const Json::Value& actual, std::initializer_list<double> expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  int i = 0;
  for (double e : expected)
    EXPECT_NEAR(actual[i++].asDouble(), e, 1e-9);
}
}  // namespace

TEST(Hash, KnownVectors)
{
  EXPECT_EQ(hashBytes(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(hashBytes("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(hashBytes("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(hashBytes(std::string(1000000, 'a')), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(StaticFiles, Embedded)
{
  EXPECT_NE(staticFile("index.html").find("app.js"), std::string_view::npos);
  EXPECT_NE(staticFile("app.js").find("processMessage"), std::string_view::npos);
  EXPECT_FALSE(staticFile("vendor/three/build/three.module.js").empty());
  EXPECT_TRUE(staticFile("geometries.json").empty());

  // The client must load nothing from outside
  EXPECT_EQ(staticFile("app.js").find("https://"), std::string_view::npos);
  EXPECT_EQ(staticFile("index.html").find("https://unpkg.com/three"), std::string_view::npos);
  EXPECT_TRUE(staticFile("../secret").empty());
}

TEST(Gltf, ShapesSceneStructure)
{
  const auto env = shapesEnv();
  const Glb glb = decodeGlb(environmentToGlb(*env, "my_scene"));
  const Json::Value& gltf = glb.json;

  EXPECT_EQ(gltf["asset"]["version"].asString(), "2.0");
  EXPECT_EQ(gltf["scene"].asInt(), 0);
  EXPECT_EQ(gltf["scenes"][0]["name"].asString(), "my_scene");
  EXPECT_EQ(gltf["scenes"][0]["nodes"][0].asInt(), 0);
  EXPECT_EQ(gltf["nodes"][0]["name"].asString(), "link_world");
  EXPECT_EQ(gltf["nodes"][0]["extras"]["tesseract_link"]["name"].asString(), "world");
  // 5 links, 2 nodes per joint for 4 joints, 4 visuals
  EXPECT_EQ(gltf["nodes"].size(), 5u + 8u + 4u);
  checkAccessors(glb);

  const Json::Value* joint = findNode(gltf, "joint_cylinder_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ((*joint)["extras"]["tesseract_joint"]["type"].asInt(), 1);
  EXPECT_EQ((*joint)["extras"]["tesseract_joint"]["name"].asString(), "cylinder_joint");
  expectVec((*joint)["extras"]["tesseract_joint"]["axis"], { 0, 1, 0 });
  EXPECT_FALSE(joint->isMember("rotation"));
  EXPECT_EQ(gltf["nodes"][(*joint)["children"][0].asInt()]["name"].asString(), "link_cylinder_link");

  EXPECT_EQ((*findNode(gltf, "joint_box_joint"))["extras"]["tesseract_joint"]["type"].asInt(), 3);
  EXPECT_EQ((*findNode(gltf, "joint_box2_joint"))["extras"]["tesseract_joint"]["type"].asInt(), 6);

  const Json::Value* parent = findNode(gltf, "jointparent_box_joint");
  ASSERT_NE(parent, nullptr);
  expectVec((*parent)["translation"], { 1, 0, 0 });
  expectVec((*parent)["rotation"], { 0, 0, 0, 1 });
  EXPECT_EQ(gltf["nodes"][(*parent)["children"][0].asInt()]["name"].asString(), "joint_box_joint");
}

TEST(Gltf, DocumentsMatchSeparateConversions)
{
  const auto env = shapesEnv();
  const SceneDocuments documents = environmentToDocuments(*env, "scene");
  EXPECT_EQ(parse(documents.gltf), parse(environmentToGltf(*env, "scene")));
  EXPECT_EQ(documents.glb, environmentToGlb(*env, "scene"));
}

TEST(Gltf, PrimitiveScaleMaterialAndSharedAccessors)
{
  const auto env = shapesEnv();
  const Glb glb = decodeGlb(environmentToGlb(*env));
  const Json::Value& gltf = glb.json;

  const auto visual = [&gltf](const std::string& link) {
    const Json::Value* found = nullptr;
    for (const auto& node : gltf["nodes"])
      if (node["name"].asString().rfind("link_" + link + "_visual_", 0) == 0)
        found = &node;
    EXPECT_NE(found, nullptr) << link;
    return found;
  };

  expectVec((*visual("box_link"))["scale"], { 0.3, 0.05, 0.1 });
  expectVec((*visual("box2_link"))["scale"], { 0.5, 1.0, 1.5 });
  expectVec((*visual("sphere_link"))["scale"], { 0.5, 0.5, 0.5 });
  expectVec((*visual("cylinder_link"))["scale"], { 0.2, 0.2, 0.3 });

  const Json::Value& box1 = gltf["meshes"][(*visual("box_link"))["mesh"].asInt()];
  const Json::Value& box2 = gltf["meshes"][(*visual("box2_link"))["mesh"].asInt()];
  EXPECT_TRUE(box1["primitives"][0]["attributes"].isMember("NORMAL"));
  EXPECT_FALSE(box1["primitives"][0]["attributes"].isMember("NORMALS"));
  EXPECT_EQ(box1["primitives"][0]["attributes"], box2["primitives"][0]["attributes"]);
  EXPECT_EQ(box1["primitives"][0]["indices"], box2["primitives"][0]["indices"]);
  // 3 shapes with 3 accessors each
  EXPECT_EQ(gltf["accessors"].size(), 9u);

  const Json::Value& red = gltf["materials"][gltf["meshes"][(*visual("cylinder_link"))["mesh"].asInt()]["primitives"][0]["material"].asInt()];
  expectVec(red["pbrMetallicRoughness"]["baseColorFactor"], { 0.8, 0, 0, 1 });
  EXPECT_DOUBLE_EQ(red["pbrMetallicRoughness"]["roughnessFactor"].asDouble(), 0.3);
  EXPECT_DOUBLE_EQ(red["pbrMetallicRoughness"]["metallicFactor"].asDouble(), 0.3);
  EXPECT_EQ(red["alphaMode"].asString(), "MASK");
  EXPECT_DOUBLE_EQ(red["alphaCutoff"].asDouble(), 0.4);
}

TEST(Gltf, GltfAndGlbCarryTheSameData)
{
  const auto env = shapesEnv();
  const Json::Value gltf = parse(environmentToGltf(*env));
  const Glb glb = decodeGlb(environmentToGlb(*env));

  const std::string prefix = "data:application/octet-stream;base64,";
  const std::string uri = gltf["buffers"][0]["uri"].asString();
  ASSERT_EQ(uri.rfind(prefix, 0), 0u);
  const std::vector<std::uint8_t> decoded = base64Decode(uri.substr(prefix.size()));
  ASSERT_EQ(decoded.size(), gltf["buffers"][0]["byteLength"].asUInt64());

  ASSERT_LE(decoded.size(), glb.bin.size());
  EXPECT_LT(glb.bin.size() - decoded.size(), 4u);
  EXPECT_TRUE(std::equal(decoded.begin(), decoded.end(), glb.bin.begin()));
  EXPECT_FALSE(glb.json["buffers"][0].isMember("uri"));
  EXPECT_EQ(gltf["nodes"], glb.json["nodes"]);
  EXPECT_EQ(gltf["accessors"], glb.json["accessors"]);
}

TEST(Gltf, TriangleMeshes)
{
  const auto env = abbEnv();
  const Glb glb = decodeGlb(environmentToGlb(*env));
  checkAccessors(glb);

  ASSERT_GT(glb.json["meshes"].size(), 0u);
  for (const auto& mesh : glb.json["meshes"])
  {
    const Json::Value& primitive = mesh["primitives"][0];
    // Mesh normals are referenced, and tesseract gives unit normals with one per vertex
    ASSERT_TRUE(primitive["attributes"].isMember("NORMAL"));
    const int positions = primitive["attributes"]["POSITION"].asInt();
    const Json::Value& normals = glb.json["accessors"][primitive["attributes"]["NORMAL"].asInt()];
    EXPECT_EQ(normals["count"].asUInt(), glb.json["accessors"][positions]["count"].asUInt());
    for (Json::ArrayIndex c = 0; c < 3; ++c)
    {
      EXPECT_LE(normals["max"][c].asDouble(), 1.0 + 1e-5);
      EXPECT_GE(normals["min"][c].asDouble(), -1.0 - 1e-5);
    }
    const int indices = primitive["indices"].asInt();
    EXPECT_EQ(glb.json["accessors"][indices]["count"].asUInt() % 3, 0u);
    EXPECT_LT(glb.json["accessors"][indices]["max"][0].asUInt(), glb.json["accessors"][positions]["count"].asUInt());
    EXPECT_TRUE(primitive.isMember("material"));
  }
  expectVec((*findNode(glb.json, "joint_joint_1"))["extras"]["tesseract_joint"]["axis"], { 0, 0, 1 });
}

TEST(Gltf, EmbeddedTextures)
{
  constexpr char URDF[] = R"(
  <robot name="mesh_viewer" tesseract:make_convex="true" xmlns:tesseract="http://tesseract">
    <link name="world"/>
    <link name="mesh_link">
      <visual><geometry>
        <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.glb"/>
      </geometry></visual>
    </link>
    <joint name="mesh_joint" type="fixed"><parent link="world"/><child link="mesh_link"/></joint>
  </robot>)";
  Environment env;
  ASSERT_TRUE(env.init(std::string(URDF), std::make_shared<GeneralResourceLocator>()));

  const Glb glb = decodeGlb(environmentToGlb(env));
  checkAccessors(glb);
  ASSERT_GT(glb.json["images"].size(), 0u);

  for (const auto& image : glb.json["images"])
  {
    const Json::Value& view = glb.json["bufferViews"][image["bufferView"].asInt()];
    const std::size_t offset = view["byteOffset"].asUInt64();
    const bool png = glb.bin[offset] == 0x89 && glb.bin[offset + 1] == 'P';
    const bool jpeg = glb.bin[offset] == 0xFF && glb.bin[offset + 1] == 0xD8;
    EXPECT_TRUE(png || jpeg);
    EXPECT_EQ(image["mimeType"].asString(), png ? "image/png" : "image/jpeg");
  }

  bool has_textured_primitive = false;
  for (const auto& mesh : glb.json["meshes"])
  {
    const Json::Value& primitive = mesh["primitives"][0];
    if (!primitive["attributes"].isMember("TEXCOORD_0"))
      continue;
    has_textured_primitive = true;
    const Json::Value& pbr = glb.json["materials"][primitive["material"].asInt()]["pbrMetallicRoughness"];
    EXPECT_TRUE(pbr.isMember("baseColorTexture"));
    EXPECT_LT(pbr["baseColorTexture"]["index"].asUInt(), glb.json["textures"].size());
  }
  EXPECT_TRUE(has_textured_primitive);
}

TEST(Gltf, PbrMaterialsAndTextureSlots)
{
  constexpr char URDF[] = R"(
  <robot name="pbr_viewer" tesseract:make_convex="false" xmlns:tesseract="http://tesseract">
    <link name="world"/>
    <link name="mesh_link">
      <visual><geometry>
        <mesh filename="package://tesseract/support/meshes/tesseract_pbr_mesh.glb"/>
      </geometry></visual>
    </link>
    <joint name="mesh_joint" type="fixed"><parent link="world"/><child link="mesh_link"/></joint>
  </robot>)";
  Environment env;
  ASSERT_TRUE(env.init(std::string(URDF), std::make_shared<GeneralResourceLocator>()));
  // glTF factors pass through float in assimp
  const auto expectNear = [](const Json::Value& actual, const std::vector<double>& expected) {
    ASSERT_EQ(actual.size(), expected.size());
    for (Json::ArrayIndex i = 0; i < actual.size(); ++i)
      EXPECT_NEAR(actual[i].asDouble(), expected[i], 1e-6);
  };

  const Glb glb = decodeGlb(environmentToGlb(env));
  checkAccessors(glb);

  // Five distinct images; metallic-roughness and normal etc. are not duplicated across slots
  EXPECT_EQ(glb.json["images"].size(), 5u);
  bool has_emissive_strength = false;
  for (const auto& name : glb.json["extensionsUsed"])
    has_emissive_strength |= name.asString() == "KHR_materials_emissive_strength";
  EXPECT_TRUE(has_emissive_strength);

  const Json::Value* full = nullptr;
  const Json::Value* full_primitive = nullptr;
  const Json::Value* mask = nullptr;
  for (const auto& mesh : glb.json["meshes"])
  {
    const Json::Value& primitive = mesh["primitives"][0];
    const Json::Value& material = glb.json["materials"][primitive["material"].asInt()];
    if (material.isMember("normalTexture"))
    {
      full = &material;
      full_primitive = &primitive;
    }
    else if (material["alphaMode"].asString() == "MASK")
    {
      mask = &material;
    }
    // Normals from the mesh file are attached to the primitive
    EXPECT_TRUE(primitive["attributes"].isMember("NORMAL"));
  }
  ASSERT_NE(full, nullptr);
  ASSERT_NE(mask, nullptr);

  EXPECT_EQ((*full)["alphaMode"].asString(), "BLEND");
  EXPECT_TRUE((*full)["doubleSided"].asBool());
  expectNear((*full)["pbrMetallicRoughness"]["baseColorFactor"], { 0.9, 0.8, 0.7, 0.6 });
  EXPECT_NEAR((*full)["pbrMetallicRoughness"]["metallicFactor"].asDouble(), 0.75, 1e-6);
  EXPECT_NEAR((*full)["pbrMetallicRoughness"]["roughnessFactor"].asDouble(), 0.25, 1e-6);
  expectNear((*full)["emissiveFactor"], { 1.0, 0.5, 0.25 });
  EXPECT_NEAR((*full)["extensions"]["KHR_materials_emissive_strength"]["emissiveStrength"].asDouble(), 2.5, 1e-6);
  EXPECT_NEAR((*full)["normalTexture"]["scale"].asDouble(), 0.5, 1e-6);
  EXPECT_NEAR((*full)["occlusionTexture"]["strength"].asDouble(), 0.8, 1e-6);

  // Occlusion samples the second UV set, everything else the first
  EXPECT_EQ((*full)["pbrMetallicRoughness"]["baseColorTexture"]["texCoord"].asInt(), 0);
  EXPECT_EQ((*full)["pbrMetallicRoughness"]["metallicRoughnessTexture"]["texCoord"].asInt(), 0);
  EXPECT_EQ((*full)["normalTexture"]["texCoord"].asInt(), 0);
  EXPECT_EQ((*full)["emissiveTexture"]["texCoord"].asInt(), 0);
  EXPECT_EQ((*full)["occlusionTexture"]["texCoord"].asInt(), 1);
  EXPECT_TRUE((*full_primitive)["attributes"].isMember("TEXCOORD_0"));
  EXPECT_TRUE((*full_primitive)["attributes"].isMember("TEXCOORD_1"));

  // The base color sampler is clamp / mirrored repeat with nearest filtering
  const Json::Value& base_texture =
      glb.json["textures"][(*full)["pbrMetallicRoughness"]["baseColorTexture"]["index"].asInt()];
  const Json::Value& sampler = glb.json["samplers"][base_texture["sampler"].asInt()];
  EXPECT_EQ(sampler["wrapS"].asInt(), 33071);
  EXPECT_EQ(sampler["wrapT"].asInt(), 33648);
  EXPECT_EQ(sampler["magFilter"].asInt(), 9728);
  EXPECT_EQ(sampler["minFilter"].asInt(), 9728);

  EXPECT_NEAR((*mask)["alphaCutoff"].asDouble(), 0.3, 1e-6);
  EXPECT_FALSE(mask->isMember("doubleSided"));
}

TEST(Gltf, UnsupportedGeometryIsSkippedAndKeepsNumbering)
{
  Environment env;
  ASSERT_TRUE(env.init(std::string(SHAPES_URDF), std::make_shared<GeneralResourceLocator>()));
  // A capsule visual has no entry in geometries.json
  tesseract::scene_graph::Link link(LinkId("capsule_link"));
  auto capsule_visual = std::make_shared<tesseract::scene_graph::Visual>();
  capsule_visual->geometry = std::make_shared<tesseract::geometry::Capsule>(0.1, 0.2);
  link.visual.push_back(capsule_visual);
  auto box_visual = std::make_shared<tesseract::scene_graph::Visual>();
  box_visual->geometry = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  link.visual.push_back(box_visual);
  ASSERT_TRUE(env.applyCommand(std::make_shared<tesseract::environment::AddLinkCommand>(link)));

  const Glb glb = decodeGlb(environmentToGlb(env));
  ASSERT_NE(findNode(glb.json, "link_capsule_link_visual_2"), nullptr);
  for (const auto& node : glb.json["nodes"])
    EXPECT_NE(node["name"].asString(), "link_capsule_link_visual_1");
}

TEST(Gltf, TrajectoryAnimation)
{
  const auto env = shapesEnv();
  JointTrajectory trajectory;
  const std::vector<JointId> joints{ JointId("cylinder_joint"), JointId("box_joint") };
  JointState a(joints, Eigen::Vector2d(0.0, 0.5));
  a.time = 0.0;
  JointState b(joints, Eigen::Vector2d(EIGEN_PI / 2.0, 1.0));
  b.time = 2.0;
  trajectory.states = { a, b };

  const Json::Value gltf = parse(environmentToGltf(*env, "", &trajectory));
  const std::string prefix = "data:application/octet-stream;base64,";
  const std::vector<std::uint8_t> bin = base64Decode(gltf["buffers"][0]["uri"].asString().substr(prefix.size()));

  ASSERT_EQ(gltf["animations"].size(), 1u);
  const Json::Value& animation = gltf["animations"][0];
  ASSERT_EQ(animation["channels"].size(), 2u);
  ASSERT_EQ(animation["samplers"].size(), 2u);

  const auto joint_node = [&gltf](const char* name) {
    for (Json::ArrayIndex i = 0; i < gltf["nodes"].size(); ++i)
      if (gltf["nodes"][i]["name"].asString() == name)
        return static_cast<int>(i);
    return -1;
  };

  EXPECT_EQ(animation["channels"][0]["target"]["path"].asString(), "rotation");
  EXPECT_EQ(animation["channels"][0]["target"]["node"].asInt(), joint_node("joint_cylinder_joint"));
  EXPECT_EQ(animation["channels"][1]["target"]["path"].asString(), "translation");
  EXPECT_EQ(animation["channels"][1]["target"]["node"].asInt(), joint_node("joint_box_joint"));

  const auto times = readAccessor<float>(gltf, bin, animation["samplers"][0]["input"].asInt());
  ASSERT_EQ(times.size(), 2u);
  EXPECT_FLOAT_EQ(times[1], 2.0F);

  // 90 degrees about Y as x, y, z, w
  const auto rotations = readAccessor<float>(gltf, bin, animation["samplers"][0]["output"].asInt());
  ASSERT_EQ(rotations.size(), 8u);
  EXPECT_NEAR(rotations[5], std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(rotations[7], std::sqrt(0.5), 1e-6);

  const auto translations = readAccessor<float>(gltf, bin, animation["samplers"][1]["output"].asInt());
  ASSERT_EQ(translations.size(), 6u);
  EXPECT_FLOAT_EQ(translations[2], 0.5F);
  EXPECT_FLOAT_EQ(translations[5], 1.0F);
}

TEST(Gltf, TrajectoryErrors)
{
  const auto env = shapesEnv();
  JointTrajectory unknown;
  unknown.states.emplace_back(std::vector<JointId>{ JointId("nope") }, Eigen::VectorXd::Zero(1));
  EXPECT_THROW(environmentToGltf(*env, "", &unknown), std::invalid_argument);

  JointTrajectory fixed;
  fixed.states.emplace_back(std::vector<JointId>{ JointId("box2_joint") }, Eigen::VectorXd::Zero(1));
  EXPECT_THROW(environmentToGltf(*env, "", &fixed), std::invalid_argument);

  EXPECT_THROW(jointTrajectoryToList(JointTrajectory{}), std::invalid_argument);
}

TEST(Trajectory, JointTrajectoryJson)
{
  const std::vector<JointId> joints{ JointId("a"), JointId("b") };
  JointTrajectory trajectory;
  JointState first(joints, Eigen::Vector2d(1, 2));
  first.time = 0.5;
  trajectory.states.push_back(first);

  const Json::Value json = parse(jointTrajectoryToJson(trajectory));
  EXPECT_TRUE(json["use_time"].asBool());
  EXPECT_EQ(json["loop_time"].asDouble(), 20);
  EXPECT_EQ(json["joint_names"][1].asString(), "b");
  expectVec(json["trajectory"][0], { 1, 2, 0.5 });

  trajectory.states.emplace_back(std::vector<JointId>{ JointId("b"), JointId("a") }, Eigen::Vector2d(1, 2));
  EXPECT_THROW(jointTrajectoryToJson(trajectory), std::invalid_argument);
}

TEST(Trajectory, RowsAreValidated)
{
  EXPECT_THROW(trajectoryListToJson(TrajectoryList{ { "a" }, { { 1.0 } } }), std::invalid_argument);
}

TEST(Trajectory, StaticJointPositions)
{
  const Json::Value json = parse(jointPositionsToTrajectoryJson({ "a", "b" }, Eigen::Vector2d(0.1, 0.2)));
  EXPECT_TRUE(json["use_time"].asBool());
  EXPECT_EQ(json["loop_time"].asDouble(), 10000);
  ASSERT_EQ(json["trajectory"].size(), 2u);
  expectVec(json["trajectory"][0], { 0.1, 0.2, 0 });
  expectVec(json["trajectory"][1], { 0.1, 0.2, 1e6 });

  EXPECT_THROW(jointPositionsToTrajectoryJson({}, Eigen::VectorXd{}), std::invalid_argument);
  EXPECT_THROW(jointPositionsToTrajectoryJson({ "a" }, Eigen::Vector2d(1, 2)), std::invalid_argument);
}

TEST(Trajectory, SceneStateIsSortedByName)
{
  tesseract::scene_graph::SceneState state;
  state.joints[JointId("zeta")] = 3;
  state.joints[JointId("alpha")] = 1;
  state.joints[JointId("mid")] = 2;
  const Json::Value json = parse(sceneStateToTrajectoryJson(state));
  EXPECT_EQ(json["joint_names"][0].asString(), "alpha");
  EXPECT_EQ(json["joint_names"][2].asString(), "zeta");
  expectVec(json["trajectory"][0], { 1, 2, 3, 0 });
}

TEST(Trajectory, FramesMatchForwardKinematics)
{
  const auto env = abbEnv();
  const auto group_joints = env->getJointGroup("manipulator")->getJointIds();

  // Reversed, to prove that columns are matched by name
  TrajectoryList list;
  std::unordered_map<JointId, double> values;
  double q = 0.1;
  for (auto it = group_joints.rbegin(); it != group_joints.rend(); ++it, q += 0.1)
  {
    list.joint_names.push_back(it->name());
    values[*it] = q;
  }
  std::vector<double> row;
  for (const auto& name : list.joint_names)
    row.push_back(values.at(JointId(name)));
  row.push_back(1.5);
  list.trajectory.push_back(row);

  const auto frames = trajectoryToFrames(*env, "manipulator", "tool0", list);
  ASSERT_EQ(frames.size(), 1u);
  const auto expected = env->getState(values).link_transforms.at(LinkId("tool0"));
  EXPECT_TRUE(frames[0].isApprox(expected, 1e-9));

  EXPECT_THROW(trajectoryToFrames(*env, "manipulator", "not_a_link", list), std::invalid_argument);
}

namespace vis = tesseract::visualization;

TEST(Markers, JsonFields)
{
  ViewerMarker axes;
  axes.marker_type = "axes";
  axes.name = "m1";
  axes.size = 0.2;
  axes.tags = { "t" };

  ViewerMarker box;
  box.marker_type = "box";
  box.name = "m2";
  box.extents = std::array<double, 3>{ 1, 2, 3 };

  ViewerMarker lines;
  lines.marker_type = "lines";
  lines.name = "m3";
  lines.vertices = std::vector<std::array<double, 3>>{ { 0, 0, 0 }, { 1, 1, 1 } };
  lines.linewidth = 2.0;

  const Json::Value json = parse(markersToJson({ axes, box, lines }));
  ASSERT_EQ(json["markers"].size(), 3u);
  const Json::Value& a = json["markers"][0];
  EXPECT_EQ(a["marker_type"].asString(), "axes");
  EXPECT_EQ(a["parent_link_name"].asString(), "world");
  expectVec(a["position"], { 0, 0, 0 });
  expectVec(a["quaternion"], { 1, 0, 0, 0 });
  expectVec(a["color"], { 1, 1, 1, 1 });
  EXPECT_EQ(a["name"].asString(), "m1");
  EXPECT_EQ(a["tags"][0].asString(), "t");
  EXPECT_DOUBLE_EQ(a["size"].asDouble(), 0.2);
  EXPECT_FALSE(a.isMember("radius"));

  expectVec(json["markers"][1]["size"], { 1, 2, 3 });
  EXPECT_TRUE(json["markers"][1]["tags"].isArray());
  expectVec(json["markers"][2]["vertices"][1], { 1, 1, 1 });
  EXPECT_DOUBLE_EQ(json["markers"][2]["linewidth"].asDouble(), 2.0);
}

TEST(Markers, AxisUsesWxyzAndScale)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(1, 2, 3));
  pose.rotate(Eigen::AngleAxisd(EIGEN_PI, Eigen::Vector3d::UnitZ()));
  vis::AxisMarker marker(pose);
  marker.setScale(Eigen::Vector3d::Constant(0.25));
  marker.setParentLink("tool0");

  const auto out = convertMarker(marker, "base", { "ns" });
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].marker_type, "axes");
  EXPECT_EQ(out[0].parent_link_name, "tool0");
  EXPECT_DOUBLE_EQ(*out[0].size, 0.25);
  EXPECT_NEAR(std::abs(out[0].quaternion[3]), 1.0, 1e-9);  // z is the last of w, x, y, z
  EXPECT_NEAR(out[0].quaternion[0], 0.0, 1e-9);
  EXPECT_EQ(out[0].tags, std::vector<std::string>{ "ns" });
}

TEST(Markers, ArrowFromTwoPointsSpansThem)
{
  const Eigen::Vector3d a(1, 0, 0);
  const Eigen::Vector3d b(1, 0, 2);
  vis::ArrowMarker marker(a, b);
  marker.material = std::make_shared<tesseract::scene_graph::Material>("m");
  marker.material->color << 1, 0, 1, 1;

  const auto out = convertMarker(marker, "base");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].marker_type, "arrow");
  EXPECT_EQ(out[0].parent_link_name, "base");
  EXPECT_NEAR((*out[0].origin)[2], 0.0, 1e-9);
  EXPECT_NEAR((*out[0].origin)[0], 1.0, 1e-9);
  EXPECT_NEAR((*out[0].direction)[2], 1.0, 1e-9);
  EXPECT_NEAR(*out[0].length, 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(out[0].color[2], 1.0);
}

TEST(Markers, CylinderIsRotatedOntoTheClientsYAxis)
{
  vis::GeometryMarker marker(std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.4));
  const auto out = convertMarker(marker, "base");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].marker_type, "cylinder");
  EXPECT_DOUBLE_EQ(*out[0].radius, 0.1);
  EXPECT_DOUBLE_EQ(*out[0].length, 0.4);

  const Eigen::Quaterniond q(out[0].quaternion[0], out[0].quaternion[1], out[0].quaternion[2], out[0].quaternion[3]);
  EXPECT_TRUE((q * Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
}

TEST(Markers, BoxAndSphereKeepOrigin)
{
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() << 1, 2, 3;
  const auto box = convertMarker(vis::GeometryMarker(std::make_shared<tesseract::geometry::Box>(1, 2, 3), origin), "b");
  ASSERT_EQ(box.size(), 1u);
  EXPECT_EQ(box[0].marker_type, "box");
  EXPECT_EQ(*box[0].extents, (std::array<double, 3>{ 1, 2, 3 }));
  EXPECT_DOUBLE_EQ(box[0].position[2], 3);
  EXPECT_DOUBLE_EQ(box[0].quaternion[0], 1);

  const auto sphere = convertMarker(vis::GeometryMarker(std::make_shared<tesseract::geometry::Sphere>(0.5), origin), "b");
  ASSERT_EQ(sphere.size(), 1u);
  EXPECT_EQ(sphere[0].marker_type, "sphere");
}

TEST(Markers, ConeIsRotatedOntoTheClientsYAxis)
{
  const auto out = convertMarker(vis::GeometryMarker(std::make_shared<tesseract::geometry::Cone>(0.1, 0.2)), "b");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].marker_type, "cone");
  EXPECT_DOUBLE_EQ(*out[0].radius, 0.1);
  EXPECT_DOUBLE_EQ(*out[0].length, 0.2);

  const Eigen::Quaterniond q(out[0].quaternion[0], out[0].quaternion[1], out[0].quaternion[2], out[0].quaternion[3]);
  EXPECT_TRUE((q * Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
}

TEST(Markers, ToolpathLinesAndAxes)
{
  tesseract::common::VectorIsometry3d segment(3, Eigen::Isometry3d::Identity());
  segment[1].translation().x() = 1;
  segment[2].translation().x() = 2;
  tesseract::common::Toolpath toolpath{ segment };

  vis::ToolpathMarker marker(toolpath);
  auto out = convertMarker(marker, "base");
  ASSERT_EQ(out.size(), 4u);
  EXPECT_EQ(out[0].marker_type, "lines");
  ASSERT_EQ(out[0].vertices->size(), 3u);
  EXPECT_DOUBLE_EQ((*out[0].vertices)[2][0], 2);
  EXPECT_EQ(out[1].marker_type, "axes");
  EXPECT_DOUBLE_EQ(*out[1].size, 0.03);

  marker.show_axis = false;
  EXPECT_EQ(convertMarker(marker, "base").size(), 1u);
  marker.show_path = false;
  EXPECT_TRUE(convertMarker(marker, "base").empty());
}

TEST(Markers, ContactResultsColoredByDistance)
{
  namespace c = tesseract::collision;
  const LinkId a("a");
  const LinkId b("b");

  c::ContactResult result;
  result.link_ids = { a, b };
  result.nearest_points = { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1) };

  const auto color_at = [&](double distance) {
    result.distance = distance;
    vis::ContactResultsMarker marker({ a }, { result }, [](const LinkId&, const LinkId&) { return 0.05; });
    const auto out = convertMarker(marker, "base");
    EXPECT_EQ(out.size(), 1u);
    return out.at(0).color;
  };
  EXPECT_EQ(color_at(-0.01), (std::array<double, 4>{ 1, 0, 0, 1 }));
  EXPECT_EQ(color_at(0.01), (std::array<double, 4>{ 1, 1, 0, 1 }));
  EXPECT_EQ(color_at(1.0), (std::array<double, 4>{ 0, 1, 0, 1 }));

  // Both links selected: an arrow in each direction
  result.distance = 1.0;
  vis::ContactResultsMarker both({ a, b }, { result }, [](const LinkId&, const LinkId&) { return 0.05; });
  const auto arrows = convertMarker(both, "base");
  ASSERT_EQ(arrows.size(), 2u);
  EXPECT_NEAR((*arrows[0].direction)[2], 1.0, 1e-9);
  EXPECT_NEAR((*arrows[1].direction)[2], -1.0, 1e-9);
}

TEST(ViewerState, InitialResourcesAreEmpty)
{
  ViewerState state;
  EXPECT_EQ(state.sceneGltf().body, nullptr);
  EXPECT_EQ(state.sceneGltf().etag, "None");
  EXPECT_EQ(state.trajectoryJson().etag, "None");
  EXPECT_EQ(state.markersJson().body, nullptr);
}

TEST(ViewerState, MessagesFollowTheProtocol)
{
  ViewerState state;
  std::vector<std::string> messages;
  state.setMessageCallback([&](const std::shared_ptr<const std::string>& m) { messages.push_back(*m); });

  state.setScene("{\"a\":1}", "bin");
  EXPECT_EQ(state.sceneGltf().etag, hashBytes("{\"a\":1}"));
  EXPECT_EQ(state.sceneGlb().etag, hashBytes("bin"));
  ASSERT_EQ(messages.size(), 1u);
  const Json::Value refresh = parse(messages[0]);
  EXPECT_EQ(refresh["command"].asString(), "refresh_scene");
  EXPECT_FALSE(refresh.isMember("params"));

  state.setJointPositions({ "j" }, Eigen::VectorXd::Constant(1, 0.5));
  ASSERT_EQ(messages.size(), 2u);
  const Json::Value trajectory = parse(messages[1]);
  EXPECT_EQ(trajectory["command"].asString(), "joint_trajectory");
  EXPECT_EQ(trajectory["params"], parse(*state.trajectoryJson().body));
  EXPECT_EQ(trajectory["params"]["joint_names"][0].asString(), "j");

  state.addSphereMarker(Eigen::Vector3d(1, 2, 3), 0.1);
  ASSERT_EQ(messages.size(), 3u);
  const Json::Value markers = parse(messages[2]);
  EXPECT_EQ(markers["command"].asString(), "markers");
  EXPECT_EQ(markers["params"]["markers"][0]["marker_type"].asString(), "sphere");
  EXPECT_EQ(markers["params"], parse(*state.markersJson().body));
}

TEST(ViewerState, GeneratedNamesCountEveryMarker)
{
  ViewerState state;
  EXPECT_EQ(state.addAxesMarker(Eigen::Isometry3d::Identity()), "axes_marker_1");
  EXPECT_EQ(state.addAxesMarker(Eigen::Isometry3d::Identity(), 0.1, "world", "named"), "named");
  EXPECT_EQ(state.addAxesMarker(Eigen::Isometry3d::Identity()), "axes_marker_3");
}

TEST(ViewerState, UpdateNowFalseStoresWithoutSending)
{
  ViewerState state;
  int count = 0;
  state.setMessageCallback([&](const std::shared_ptr<const std::string>&) { ++count; });

  const std::string before = state.markersJson().etag;
  state.addAxesMarker(Eigen::Isometry3d::Identity(), 0.15, "world", "", {}, false);
  EXPECT_EQ(count, 0);
  EXPECT_NE(state.markersJson().etag, before);
}

TEST(ViewerState, ClearMarkers)
{
  ViewerState state;
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  state.addAxesMarker(pose, 0.1, "world", "a", { "x" });
  state.addAxesMarker(pose, 0.1, "world", "b", { "x", "y" });
  state.addAxesMarker(pose, 0.1, "world", "c", { "y" });

  // Exact list equality, as in the Python viewer: { "x", "y" } survives
  state.clearMarkersByTags({ "x" });
  Json::Value json = parse(*state.markersJson().body);
  ASSERT_EQ(json["markers"].size(), 2u);
  EXPECT_EQ(json["markers"][0]["name"].asString(), "b");

  state.clearMarkersByName("b");
  json = parse(*state.markersJson().body);
  ASSERT_EQ(json["markers"].size(), 1u);
  EXPECT_EQ(json["markers"][0]["name"].asString(), "c");

  state.clearAllMarkers();
  EXPECT_EQ(parse(*state.markersJson().body)["markers"].size(), 0u);
}

TEST(ViewerState, PlotTrajectoryFrames)
{
  ViewerState state;
  tesseract::common::VectorIsometry3d frames(2, Eigen::Isometry3d::Identity());
  frames[1].translation().z() = 1;
  state.plotTrajectoryFrames(frames, "base", { 0, 1, 0, 1 }, 0.001, true, 0.1, { "mine" });

  const Json::Value json = parse(*state.markersJson().body);
  ASSERT_EQ(json["markers"].size(), 3u);
  EXPECT_EQ(json["markers"][0]["marker_type"].asString(), "lines");
  EXPECT_EQ(json["markers"][0]["parent_link_name"].asString(), "base");
  EXPECT_EQ(json["markers"][0]["vertices"].size(), 2u);
  EXPECT_EQ(json["markers"][0]["tags"][1].asString(), "trajectory");
  EXPECT_EQ(json["markers"][1]["marker_type"].asString(), "axes");
  EXPECT_DOUBLE_EQ(json["markers"][2]["size"].asDouble(), 0.1);
}
