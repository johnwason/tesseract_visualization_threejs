// Port of tesseract_env_to_gltf.py. Node, accessor and buffer order deliberately follow the Python
// code, quirks included, so that the output keeps the structure the unchanged client was built on:
//  - primitive meshes use the non-standard attribute name "NORMALS"
//  - triangle mesh normals are written to the buffer but not referenced by the primitive
//  - JPEG textures use the mime type "image/jpg"

#include <openssl/evp.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_map>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/logging.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/mesh_material.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/viewer/conversions.h>

#include "embedded_resources.h"
#include "json_util.h"

namespace tesseract::viewer
{
namespace
{
using detail::toJsonArray;
using tesseract::scene_graph::JointType;

constexpr int COMPONENT_TYPE_UINT32 = 5125;
constexpr int COMPONENT_TYPE_FLOAT32 = 5126;
constexpr std::uint32_t GLB_MAGIC = 0x46546C67;
constexpr std::uint32_t GLB_CHUNK_TYPE_JSON = 0x4E4F534A;
constexpr std::uint32_t GLB_CHUNK_TYPE_BIN = 0x004E4942;

struct ShapeData
{
  std::vector<float> positions;
  std::vector<float> normals;
  std::vector<std::uint32_t> indices;
};

template <class T>
std::vector<T> readNumbers(const Json::Value& array)
{
  std::vector<T> out;
  out.reserve(array.size());
  for (const auto& v : array)
    out.push_back(static_cast<T>(v.asDouble()));
  return out;
}

const std::map<std::string, ShapeData>& primitiveShapes()
{
  static const std::map<std::string, ShapeData> shapes = [] {
    const std::string_view text = detail::asView(detail::geometries_json, detail::geometries_json_size);
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    std::string errors;
    if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors))
      throw std::runtime_error("Failed to parse embedded geometries.json: " + errors);

    std::map<std::string, ShapeData> out;
    for (const auto& g : root["geometries"]["vertexData"])
    {
      const std::string id = g["id"].asString();
      if (id == "cube_geometry" || id == "sphere_geometry" || id == "cylinder_geometry")
        out[id] = ShapeData{ readNumbers<float>(g["positions"]),
                             readNumbers<float>(g["normals"]),
                             readNumbers<std::uint32_t>(g["indices"]) };
    }
    return out;
  }();
  return shapes;
}

bool endsWith(const std::string& text, const std::string& suffix)
{
  return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string toLower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  return text;
}

std::string base64Encode(const std::vector<std::uint8_t>& data)
{
  if (data.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    throw std::length_error("Scene is too large to embed as base64");

  // Four output characters per three input bytes, plus the terminator EVP_EncodeBlock writes
  std::string out((data.size() + 2) / 3 * 4 + 1, '\0');
  const int written = EVP_EncodeBlock(reinterpret_cast<unsigned char*>(out.data()), data.data(), static_cast<int>(data.size()));
  out.resize(static_cast<std::size_t>(written));
  return out;
}

void appendLittleEndian(std::vector<std::uint8_t>& out, std::uint32_t value)
{
  for (int i = 0; i < 4; ++i)
    out.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

struct JointNode
{
  Json::ArrayIndex node;
  Eigen::Vector3f axis;
  JointType type;
};

struct ShapeAccessors
{
  Json::ArrayIndex positions;
  Json::ArrayIndex normals;
  Json::ArrayIndex indices;
};

class GltfBuilder
{
public:
  GltfBuilder(const tesseract::environment::Environment& env, const std::string& scene_name) : env_(env)
  {
    Json::Value scene;
    scene["nodes"].append(0);
    if (!scene_name.empty())
      scene["name"] = scene_name;

    root_["asset"]["generator"] = "Tesseract Robotics Scene Viewer C++ glTF";
    root_["asset"]["version"] = "2.0";
    root_["scene"] = 0;
    root_["scenes"].append(scene);

    for (const auto& id : env.getJointIds())
      joints_.push_back(env.getJoint(id));

    appendLink(env.getRootLinkId());
  }

  void appendTrajectory(const tesseract::common::JointTrajectory& trajectory)
  {
    const TrajectoryList list = jointTrajectoryToList(trajectory);
    const std::size_t joint_count = list.joint_names.size();

    std::vector<float> times;
    times.reserve(list.trajectory.size());
    for (const auto& row : list.trajectory)
      times.push_back(static_cast<float>(row.back()));
    const Json::ArrayIndex time_accessor = appendAccessor(times, 1);

    Json::Value animation;
    animation["channels"] = Json::Value(Json::arrayValue);
    animation["samplers"] = Json::Value(Json::arrayValue);

    for (std::size_t i = 0; i < joint_count; ++i)
    {
      const std::string& name = list.joint_names[i];
      const auto it = joint_nodes_.find(name);
      if (it == joint_nodes_.end())
        throw std::invalid_argument("Trajectory joint is not in the environment: " + name);
      const JointNode& joint = it->second;
      const Eigen::Vector3d axis = joint.axis.cast<double>();

      std::vector<float> output;
      const char* path = nullptr;
      int components = 0;
      if (joint.type == JointType::REVOLUTE || joint.type == JointType::CONTINUOUS)
      {
        path = "rotation";
        components = 4;
        for (const auto& row : list.trajectory)
        {
          const Eigen::Quaterniond q(Eigen::AngleAxisd(static_cast<double>(static_cast<float>(row[i])), axis));
          output.insert(output.end(),
                        { static_cast<float>(q.x()),
                          static_cast<float>(q.y()),
                          static_cast<float>(q.z()),
                          static_cast<float>(q.w()) });
        }
      }
      else if (joint.type == JointType::PRISMATIC)
      {
        path = "translation";
        components = 3;
        for (const auto& row : list.trajectory)
        {
          const Eigen::Vector3d v = static_cast<double>(static_cast<float>(row[i])) * axis;
          output.insert(output.end(),
                        { static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) });
        }
      }
      else
      {
        throw std::invalid_argument("Joint type cannot be animated: " + name);
      }

      Json::Value sampler;
      sampler["input"] = time_accessor;
      sampler["output"] = appendAccessor(output, components);
      sampler["interpolation"] = "LINEAR";
      animation["samplers"].append(sampler);

      Json::Value channel;
      channel["sampler"] = static_cast<Json::UInt>(i);
      channel["target"]["node"] = joint.node;
      channel["target"]["path"] = path;
      animation["channels"].append(channel);
    }

    root_["animations"].append(animation);
  }

  Json::Value& root() { return root_; }
  std::vector<std::uint8_t>& buffer() { return buffer_; }

private:
  Json::ArrayIndex appendTo(const char* key, Json::Value value)
  {
    Json::Value& list = root_[key];
    list.append(std::move(value));
    return list.size() - 1;
  }

  Json::ArrayIndex appendNode(const Eigen::Isometry3d* tf, const std::string& name, const Json::Value* extras = nullptr)
  {
    Json::Value node;
    if (tf != nullptr)
    {
      const Eigen::Quaterniond q(tf->rotation());
      const Eigen::Vector3d p = tf->translation();
      node["rotation"] = toJsonArray(std::array<double, 4>{ q.x(), q.y(), q.z(), q.w() });
      node["translation"] = toJsonArray(std::array<double, 3>{ p.x(), p.y(), p.z() });
    }
    node["name"] = name;
    if (extras != nullptr)
      node["extras"] = *extras;
    return appendTo("nodes", std::move(node));
  }

  Json::ArrayIndex appendBufferView(const void* data, std::size_t size)
  {
    Json::Value view;
    view["buffer"] = 0;
    view["byteLength"] = static_cast<Json::UInt64>(size);
    view["byteOffset"] = static_cast<Json::UInt64>(buffer_.size());
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    buffer_.insert(buffer_.end(), bytes, bytes + size);
    return appendTo("bufferViews", std::move(view));
  }

  template <class T>
  Json::ArrayIndex appendAccessor(const std::vector<T>& values, int components)
  {
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, std::uint32_t>);
    static constexpr std::array<const char*, 4> TYPES = { "SCALAR", "VEC2", "VEC3", "VEC4" };

    Json::Value accessor;
    accessor["bufferView"] = appendBufferView(values.data(), values.size() * sizeof(T));
    accessor["componentType"] = std::is_same_v<T, float> ? COMPONENT_TYPE_FLOAT32 : COMPONENT_TYPE_UINT32;
    accessor["type"] = TYPES.at(static_cast<std::size_t>(components - 1));
    accessor["count"] = static_cast<Json::UInt64>(values.size() / static_cast<std::size_t>(components));

    if (!values.empty())
    {
      std::vector<T> min(values.begin(), values.begin() + components);
      std::vector<T> max = min;
      for (std::size_t i = 0; i < values.size(); ++i)
      {
        const std::size_t c = i % static_cast<std::size_t>(components);
        min[c] = std::min(min[c], values[i]);
        max[c] = std::max(max[c], values[i]);
      }
      Json::Value json_min(Json::arrayValue);
      Json::Value json_max(Json::arrayValue);
      for (std::size_t c = 0; c < min.size(); ++c)
      {
        if constexpr (std::is_same_v<T, float>)
        {
          json_min.append(static_cast<double>(min[c]));
          json_max.append(static_cast<double>(max[c]));
        }
        else
        {
          json_min.append(static_cast<Json::UInt>(min[c]));
          json_max.append(static_cast<Json::UInt>(max[c]));
        }
      }
      accessor["min"] = json_min;
      accessor["max"] = json_max;
    }
    return appendTo("accessors", std::move(accessor));
  }

  Json::ArrayIndex appendLink(const tesseract::common::LinkId& link_id)
  {
    const auto link = env_.getLink(link_id);
    const std::string& link_name = link->getName();

    Json::Value link_extras;
    link_extras["tesseract_link"]["name"] = link_name;
    const Json::ArrayIndex link_index = appendNode(nullptr, "link_" + link_name, &link_extras);

    Json::Value children(Json::arrayValue);
    int visual_number = 0;
    for (const auto& visual : link->visual)
    {
      ++visual_number;
      appendVisual(link_name, *visual, visual_number, children);
    }

    for (const auto& joint : joints_)
    {
      if (joint->parent_link_id != link_id)
        continue;
      const std::string& joint_name = joint->getName();
      const Json::ArrayIndex parent_joint_index =
          appendNode(&joint->parent_to_joint_origin_transform, "jointparent_" + joint_name);

      Json::Value joint_extras;
      joint_extras["tesseract_joint"]["axis"] =
          toJsonArray(std::array<double, 3>{ joint->axis.x(), joint->axis.y(), joint->axis.z() });
      joint_extras["tesseract_joint"]["type"] = static_cast<int>(joint->type);
      joint_extras["tesseract_joint"]["name"] = joint_name;
      const Json::ArrayIndex joint_index = appendNode(nullptr, "joint_" + joint_name, &joint_extras);
      joint_nodes_[joint_name] = JointNode{ joint_index, joint->axis.cast<float>(), joint->type };

      Json::Value parent_children(Json::arrayValue);
      parent_children.append(joint_index);
      root_["nodes"][parent_joint_index]["children"] = parent_children;

      const Json::ArrayIndex child_link_index = appendLink(joint->child_link_id);
      Json::Value joint_children(Json::arrayValue);
      joint_children.append(child_link_index);
      root_["nodes"][joint_index]["children"] = joint_children;

      children.append(parent_joint_index);
    }

    if (!children.empty())
      root_["nodes"][link_index]["children"] = children;
    return link_index;
  }

  void appendVisual(const std::string& link_name,
                    const tesseract::scene_graph::Visual& visual,
                    int visual_number,
                    Json::Value& children)
  {
    namespace g = tesseract::geometry;
    const std::string visual_name = "link_" + link_name + "_visual_" + visual.name + std::to_string(visual_number);
    const g::Geometry* geometry = visual.geometry.get();

    if (const auto* compound = dynamic_cast<const g::CompoundMesh*>(geometry))
    {
      int mesh_number = 0;
      for (const auto& mesh : compound->getMeshes())
      {
        const std::string mesh_name = visual_name + "_mesh_" + std::to_string(mesh_number++);
        const Json::ArrayIndex node = appendNode(&visual.origin, mesh_name);
        Json::Value material = Json::nullValue;
        const Json::ArrayIndex mesh_index = appendPolygonMesh(root_["nodes"][node], mesh_name, *mesh, material);
        applyMaterial(mesh_index, mesh_name, material, visual);
        root_["nodes"][node]["mesh"] = mesh_index;
        children.append(node);
      }
      return;
    }

    const auto* polygon_mesh = dynamic_cast<const g::PolygonMesh*>(geometry);
    const auto* box = dynamic_cast<const g::Box*>(geometry);
    const auto* sphere = dynamic_cast<const g::Sphere*>(geometry);
    const auto* cylinder = dynamic_cast<const g::Cylinder*>(geometry);
    if (polygon_mesh == nullptr && box == nullptr && sphere == nullptr && cylinder == nullptr)
    {
      // The Python viewer raised UnboundLocalError here. A single unsupported visual should not
      // take down plotting of the rest of the environment.
      TESSERACT_LOG_WARN("tesseract_viewer: skipping visual '{}': unsupported geometry type", visual_name);
      return;
    }

    const Json::ArrayIndex node = appendNode(&visual.origin, visual_name);
    Json::Value material = Json::nullValue;
    Json::ArrayIndex mesh_index = 0;

    if (polygon_mesh != nullptr)
    {
      mesh_index = appendPolygonMesh(root_["nodes"][node], visual_name, *polygon_mesh, material);
    }
    else if (box != nullptr)
    {
      mesh_index = appendShapeMesh("cube_geometry", visual_name);
      root_["nodes"][node]["scale"] =
          toJsonArray(std::array<double, 3>{ 0.5 * box->getX(), 0.5 * box->getY(), 0.5 * box->getZ() });
    }
    else if (sphere != nullptr)
    {
      mesh_index = appendShapeMesh("sphere_geometry", visual_name);
      const double r = sphere->getRadius();
      root_["nodes"][node]["scale"] = toJsonArray(std::array<double, 3>{ r, r, r });
    }
    else
    {
      mesh_index = appendShapeMesh("cylinder_geometry", visual_name);
      const double r = cylinder->getRadius();
      root_["nodes"][node]["scale"] = toJsonArray(std::array<double, 3>{ r, r, 0.5 * cylinder->getLength() });
    }

    applyMaterial(mesh_index, visual_name, material, visual);
    root_["nodes"][node]["mesh"] = mesh_index;
    children.append(node);
  }

  Json::ArrayIndex appendPolygonMesh(Json::Value& node,
                                     const std::string& visual_name,
                                     const tesseract::geometry::PolygonMesh& mesh,
                                     Json::Value& material)
  {
    const auto& vertices = *mesh.getVertices();
    std::vector<float> positions;
    positions.reserve(vertices.size() * 3);
    for (const auto& v : vertices)
      positions.insert(positions.end(), { static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) });

    // Faces are [n, i0, ..., i(n-1)]. Meshes are triangles in practice; other polygons are fanned.
    const Eigen::VectorXi& faces = *mesh.getFaces();
    std::vector<std::uint32_t> indices;
    indices.reserve(static_cast<std::size_t>(faces.size()) / 4 * 3);
    for (Eigen::Index i = 0; i < faces.size(); i += faces[i] + 1)
      for (int k = 1; k + 1 < faces[i]; ++k)
        indices.insert(indices.end(),
                       { static_cast<std::uint32_t>(faces[i + 1]),
                         static_cast<std::uint32_t>(faces[i + 1 + k]),
                         static_cast<std::uint32_t>(faces[i + 2 + k]) });

    Json::Value primitive;
    primitive["attributes"]["POSITION"] = appendAccessor(positions, 3);
    primitive["indices"] = appendAccessor(indices, 1);

    if (const auto& normals = mesh.getNormals(); normals != nullptr)
    {
      std::vector<float> flat;
      flat.reserve(normals->size() * 3);
      for (const auto& n : *normals)
        flat.insert(flat.end(), { static_cast<float>(n.x()), static_cast<float>(n.y()), static_cast<float>(n.z()) });
      appendAccessor(flat, 3);
    }

    node["scale"] = toJsonArray(std::array<double, 3>{ 1.0, 1.0, 1.0 });

    const auto resource = mesh.getResource();
    if (resource != nullptr && !endsWith(toLower(resource->getUrl()), ".stl") && mesh.getMaterial() != nullptr)
    {
      const auto mesh_material = mesh.getMaterial();
      material["name"] = "material_" + visual_name;
      material["alphaMode"] = "MASK";
      material["alphaCutoff"] = 0.4;

      const Eigen::Vector4d color = mesh_material->getBaseColorFactor();
      Json::Value& pbr = material["pbrMetallicRoughness"];
      pbr["baseColorFactor"] = toJsonArray(std::array<double, 4>{ color[0], color[1], color[2], color[3] });
      pbr["roughnessFactor"] = mesh_material->getRoughnessFactor();
      pbr["metallicFactor"] = mesh_material->getMetallicFactor();

      if (const auto& textures = mesh.getTextures(); textures != nullptr && !textures->empty())
      {
        const auto& texture = textures->front();
        const auto image_resource = texture->getTextureImage();
        const std::vector<std::uint8_t> image_bytes = image_resource->getResourceContents();

        Json::Value image;
        image["mimeType"] = endsWith(image_resource->getUrl(), "png") ? "image/png" : "image/jpg";
        image["bufferView"] = appendBufferView(image_bytes.data(), image_bytes.size());
        const Json::ArrayIndex image_index = appendTo("images", std::move(image));

        Json::Value gltf_texture;
        gltf_texture["source"] = image_index;
        pbr["baseColorTexture"]["index"] = appendTo("textures", std::move(gltf_texture));
        pbr["baseColorTexture"]["texCoord"] = 0;

        const auto& uvs = *texture->getUVs();
        std::vector<float> flat_uvs;
        flat_uvs.reserve(uvs.size() * 2);
        for (const auto& uv : uvs)
          flat_uvs.insert(flat_uvs.end(), { static_cast<float>(uv.x()), static_cast<float>(1.0 - uv.y()) });
        primitive["attributes"]["TEXCOORD_0"] = appendAccessor(flat_uvs, 2);
      }
    }

    Json::Value gltf_mesh;
    gltf_mesh["primitives"].append(std::move(primitive));
    gltf_mesh["name"] = visual_name + "_mesh";
    return appendTo("meshes", std::move(gltf_mesh));
  }

  Json::ArrayIndex appendShapeMesh(const std::string& shape_name, const std::string& visual_name)
  {
    auto it = shape_accessors_.find(shape_name);
    if (it == shape_accessors_.end())
    {
      const ShapeData& shape = primitiveShapes().at(shape_name);
      ShapeAccessors accessors{};
      accessors.positions = appendAccessor(shape.positions, 3);
      accessors.normals = appendAccessor(shape.normals, 3);
      accessors.indices = appendAccessor(shape.indices, 1);
      it = shape_accessors_.emplace(shape_name, accessors).first;
    }

    Json::Value primitive;
    primitive["attributes"]["POSITION"] = it->second.positions;
    primitive["attributes"]["NORMALS"] = it->second.normals;
    primitive["indices"] = it->second.indices;

    Json::Value mesh;
    mesh["primitives"].append(std::move(primitive));
    mesh["name"] = visual_name + "_mesh";
    return appendTo("meshes", std::move(mesh));
  }

  void applyMaterial(Json::ArrayIndex mesh_index,
                     const std::string& visual_name,
                     Json::Value material,
                     const tesseract::scene_graph::Visual& visual)
  {
    if (material.isNull())
    {
      material["name"] = "material_" + visual_name;
      material["alphaMode"] = "MASK";
      material["alphaCutoff"] = 0.4;
      Eigen::Vector4d color(0.5, 0.5, 0.5, 1.0);
      if (visual.material != nullptr)
        color = visual.material->color;
      Json::Value& pbr = material["pbrMetallicRoughness"];
      pbr["baseColorFactor"] = toJsonArray(std::array<double, 4>{ color[0], color[1], color[2], color[3] });
      pbr["roughnessFactor"] = 0.3;
      pbr["metallicFactor"] = 0.3;
    }
    root_["meshes"][mesh_index]["primitives"][0]["material"] = appendTo("materials", std::move(material));
  }

  const tesseract::environment::Environment& env_;
  Json::Value root_;
  std::vector<std::uint8_t> buffer_;
  std::vector<std::shared_ptr<const tesseract::scene_graph::Joint>> joints_;
  std::unordered_map<std::string, JointNode> joint_nodes_;
  std::map<std::string, ShapeAccessors> shape_accessors_;
};

}  // namespace

std::string environmentToGltf(const tesseract::environment::Environment& env,
                              const std::string& scene_name,
                              const tesseract::common::JointTrajectory* trajectory)
{
  GltfBuilder builder(env, scene_name);
  if (trajectory != nullptr)
    builder.appendTrajectory(*trajectory);

  Json::Value gltf_buffer;
  gltf_buffer["byteLength"] = static_cast<Json::UInt64>(builder.buffer().size());
  gltf_buffer["uri"] = "data:application/octet-stream;base64," + base64Encode(builder.buffer());
  builder.root()["buffers"].append(std::move(gltf_buffer));
  return detail::writeJson(builder.root());
}

std::vector<std::uint8_t> environmentToGlb(const tesseract::environment::Environment& env, const std::string& scene_name)
{
  GltfBuilder builder(env, scene_name);

  std::vector<std::uint8_t>& binary = builder.buffer();
  while (binary.size() % 4 != 0)
    binary.push_back(0);

  Json::Value gltf_buffer;
  gltf_buffer["byteLength"] = static_cast<Json::UInt64>(binary.size());
  builder.root()["buffers"].append(std::move(gltf_buffer));

  std::string json = detail::writeJson(builder.root());
  while (json.size() % 4 != 0)
    json.push_back(' ');

  const std::size_t total = 12 + 8 + json.size() + 8 + binary.size();
  if (total > std::numeric_limits<std::uint32_t>::max())
    throw std::length_error("Scene is too large for a GLB container");

  std::vector<std::uint8_t> out;
  out.reserve(total);
  appendLittleEndian(out, GLB_MAGIC);
  appendLittleEndian(out, 2);
  appendLittleEndian(out, static_cast<std::uint32_t>(total));
  appendLittleEndian(out, static_cast<std::uint32_t>(json.size()));
  appendLittleEndian(out, GLB_CHUNK_TYPE_JSON);
  out.insert(out.end(), json.begin(), json.end());
  appendLittleEndian(out, static_cast<std::uint32_t>(binary.size()));
  appendLittleEndian(out, GLB_CHUNK_TYPE_BIN);
  out.insert(out.end(), binary.begin(), binary.end());
  return out;
}

}  // namespace tesseract::viewer
