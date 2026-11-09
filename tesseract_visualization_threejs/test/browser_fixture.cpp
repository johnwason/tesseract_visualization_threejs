// Test double for tesseract_visualization_threejs/test/browser/viewer.spec.js. Runs the real ViewerState and
// ViewerServer and takes one command per line on stdin, answering with one JSON line on stdout.
// Link poses are printed from tesseract's own state solver, so the browser tests can compare what
// three.js built against an independent source.

#include <cmath>
#include <iostream>
#include <sstream>

#include <json/json.h>

#include <tesseract/collision/types.h>
#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/visualization/threejs/viewer_server.h>
#include <tesseract/visualization/threejs/viewer_state.h>
#include <tesseract/visualization/markers/arrow_marker.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/markers/contact_results_marker.h>
#include <tesseract/visualization/markers/geometry_marker.h>
#include <tesseract/visualization/markers/toolpath_marker.h>

using namespace tesseract::visualization::threejs;
using tesseract::common::GeneralResourceLocator;
using tesseract::common::JointId;
using tesseract::common::LinkId;
using tesseract::environment::Environment;
namespace vis = tesseract::visualization;
namespace geom = tesseract::geometry;

namespace
{
const std::string SHAPES = R"(
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
    <parent link="world"/><child link="box_link"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit effort="0" lower="-2" upper="2" velocity="6"/>
  </joint>
  <link name="sphere_link">
    <visual><geometry><sphere radius="0.5"/></geometry></visual>
  </link>
  <joint name="sphere_joint" type="fixed">
    <parent link="world"/><child link="sphere_link"/><origin xyz="0 2 0"/>
  </joint>
)";
const std::string EXTRA_LINK = R"(
  <link name="extra_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
  <joint name="extra_joint" type="fixed"><parent link="world"/><child link="extra_link"/><origin xyz="0 0 3"/></joint>
)";
const std::string TEXTURED = R"(
<robot name="mesh_viewer" tesseract:make_convex="true" xmlns:tesseract="http://tesseract">
  <link name="world"/>
  <link name="mesh_gltf2_link"><visual><geometry>
    <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.glb"/></geometry></visual></link>
  <joint name="mesh_gltf2_joint" type="fixed"><parent link="world"/><child link="mesh_gltf2_link"/><origin xyz="0 -1 0.25"/></joint>
  <link name="mesh_dae_link"><visual><geometry>
    <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.dae"/></geometry></visual></link>
  <joint name="mesh_dae_joint" type="fixed"><parent link="world"/><child link="mesh_dae_link"/><origin xyz="0 1 0.25"/></joint>
</robot>
)";

std::string readResource(const std::string& url)
{
  GeneralResourceLocator locator;
  auto resource = locator.locateResource(url);
  if (resource == nullptr)
    throw std::runtime_error("Cannot resolve " + url);
  const auto bytes = resource->getResourceContents();
  return { bytes.begin(), bytes.end() };
}

std::shared_ptr<Environment> loadEnvironment(const std::string& name)
{
  auto env = std::make_shared<Environment>();
  auto locator = std::make_shared<GeneralResourceLocator>();
  bool ok = false;
  if (name == "shapes")
    ok = env->init(SHAPES + "</robot>", locator);
  else if (name == "shapes_extra")
    ok = env->init(SHAPES + EXTRA_LINK + "</robot>", locator);
  else if (name == "textured")
    ok = env->init(TEXTURED, locator);
  else if (name == "abb")
    ok = env->init(readResource("package://tesseract/support/urdf/abb_irb2400.urdf"),
                   readResource("package://tesseract/support/urdf/abb_irb2400.srdf"),
                   locator);
  if (!ok)
    throw std::runtime_error("Failed to load environment " + name);
  return env;
}

Json::Value linkPoses(const Environment& env, const tesseract::scene_graph::SceneState& state)
{
  Json::Value out;
  for (const auto& [id, tf] : state.link_transforms)
  {
    const Eigen::Vector3d p = tf.translation();
    const Eigen::Quaterniond q(tf.rotation());
    Json::Value pose;
    for (double v : { p.x(), p.y(), p.z() })
      pose["position"].append(v);
    for (double v : { q.x(), q.y(), q.z(), q.w() })
      pose["quaternion"].append(v);
    out[id.name()] = pose;
  }
  (void)env;
  return out;
}

void reply(Json::Value value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  std::cout << Json::writeString(builder, value) << std::endl;
}

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() << x, y, z;
  return t;
}

/** @brief One of every marker tesseract has, converted exactly as the plugin converts them */
std::vector<ViewerMarker> allMarkers(const std::string& root)
{
  std::vector<ViewerMarker> out;
  const auto add = [&](const vis::Marker& marker, const std::string& name) {
    auto converted = convertMarker(marker, root, { "fixture" });
    int part = 0;
    for (auto& m : converted)
    {
      m.name = name + (converted.size() > 1 ? "_" + std::to_string(part++) : "");
      out.push_back(std::move(m));
    }
  };

  vis::AxisMarker axis(translation(1, 0, 0));
  axis.setScale(Eigen::Vector3d::Constant(0.2));
  add(axis, "axis");

  vis::ArrowMarker arrow(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 2));
  arrow.material = std::make_shared<tesseract::scene_graph::Material>("magenta");
  arrow.material->color << 1, 0, 1, 1;
  add(arrow, "arrow");

  Eigen::Isometry3d rotated = translation(3, 0, 0);
  rotated.rotate(Eigen::AngleAxisd(EIGEN_PI / 2.0, Eigen::Vector3d::UnitZ()));
  add(vis::GeometryMarker(std::make_shared<geom::Box>(1, 2, 3), rotated), "box");
  add(vis::GeometryMarker(std::make_shared<geom::Sphere>(0.25), translation(0, 3, 0)), "sphere");
  add(vis::GeometryMarker(std::make_shared<geom::Cylinder>(0.1, 0.4), translation(0, 1, 0)), "cylinder");

  // The client ignores capsules and must keep going
  add(vis::GeometryMarker(std::make_shared<geom::Capsule>(0.1, 0.4), translation(0, 4, 0)), "capsule");
  add(vis::GeometryMarker(std::make_shared<geom::Cone>(0.1, 0.4), translation(5, 0, 0)), "cone");
  add(vis::GeometryMarker(std::make_shared<geom::Sphere>(0.1), translation(0, 6, 0)), "after_unsupported");

  tesseract::common::VectorIsometry3d segment{ translation(0, 0, 1), translation(1, 0, 1), translation(1, 1, 1) };
  add(vis::ToolpathMarker(tesseract::common::Toolpath{ segment }), "toolpath");

  tesseract::collision::ContactResult contact;
  contact.link_ids = { LinkId("a"), LinkId("b") };
  contact.distance = -0.01;
  contact.nearest_points = { Eigen::Vector3d(2, 2, 0), Eigen::Vector3d(2, 2, 1) };
  add(vis::ContactResultsMarker({ LinkId("a") }, { contact }, [](const LinkId&, const LinkId&) { return 0.05; }),
      "contact");
  return out;
}
}  // namespace

int main()
{
  auto state = std::make_shared<ViewerState>();
  ViewerServer server(state, "127.0.0.1", 0);
  server.start();
  std::shared_ptr<Environment> env;

  Json::Value started;
  started["port"] = server.port();
  reply(started);

  std::string line;
  while (std::getline(std::cin, line))
  {
    std::istringstream in(line);
    std::string command;
    in >> command;
    Json::Value out;
    try
    {
      if (command == "env" || command == "env_keep")
      {
        std::string name;
        in >> name;
        env = loadEnvironment(name);
        state->setEnvironment(*env);
        if (command == "env")  // env_keep leaves the markers and trajectory as they were
        {
          state->clearAllMarkers();
          std::vector<std::string> active;
          for (const auto& id : env->getActiveJointIds())
            active.push_back(id.name());
          if (!active.empty())
            state->setJointPositions(active, Eigen::VectorXd::Zero(static_cast<Eigen::Index>(active.size())));
        }
        out["active_joints"] = Json::Value(Json::arrayValue);
        for (const auto& id : env->getActiveJointIds())
          out["active_joints"].append(id.name());
        out["links"] = static_cast<Json::UInt>(env->getLinkIds().size());
        out["poses"] = linkPoses(*env, env->getState());
      }
      else if (command == "joints")
      {
        std::vector<std::string> names;
        tesseract::scene_graph::SceneState::JointValues values;
        for (const auto& id : env->getActiveJointIds())
        {
          double q = 0;
          in >> q;
          names.push_back(id.name());
          values[id] = q;
        }
        Eigen::VectorXd positions(static_cast<Eigen::Index>(names.size()));
        for (std::size_t i = 0; i < names.size(); ++i)
          positions[static_cast<Eigen::Index>(i)] = values.at(JointId(names[i]));
        state->setJointPositions(names, positions);
        out["poses"] = linkPoses(*env, env->getState(values));
      }
      else if (command == "trajectory")
      {
        TrajectoryList list;
        const auto ids = env->getActiveJointIds();
        for (const auto& id : ids)
          list.joint_names.push_back(id.name());
        for (double t : { 0.0, 0.5, 1.0, 1.5, 2.0 })
        {
          std::vector<double> row(ids.size(), 0.4 * t);
          row.push_back(t);
          list.trajectory.push_back(std::move(row));
        }
        state->setTrajectoryList(list);
      }
      else if (command == "markers")
      {
        const auto markers = allMarkers(env->getRootLinkId().name());
        state->addMarkers(markers);
        out["names"] = Json::Value(Json::arrayValue);
        for (const auto& marker : markers)
          out["names"].append(marker.name);
      }
      else if (command == "marker_on")
      {
        std::string link;
        std::string name;
        in >> link >> name;
        state->addSphereMarker(Eigen::Vector3d(0, 0, 4), 0.1, { 1, 0, 0, 1 }, link, name);
      }
      else if (command == "stale_trajectory")
      {
        std::string joint;
        in >> joint;
        state->setTrajectoryList(TrajectoryList{ { joint }, { { 0.0, 0.0 }, { 1.0, 1.0 } } });
      }
      else if (command == "clear")
      {
        state->clearAllMarkers();
      }
      else if (command == "quit")
      {
        break;
      }
      else
      {
        throw std::runtime_error("Unknown command: " + command);
      }
      out["ok"] = true;
    }
    catch (const std::exception& e)
    {
      out["ok"] = false;
      out["error"] = e.what();
    }
    reply(out);
  }
  server.stop();
  return 0;
}
