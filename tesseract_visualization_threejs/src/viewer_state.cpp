#include <algorithm>

#include <tesseract/common/joint_state.h>
#include <tesseract/environment/environment.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/visualization/threejs/viewer_state.h>

namespace tesseract::visualization::threejs
{
namespace
{
ViewerResource makeResource(std::string body)
{
  ViewerResource resource;
  resource.etag = hashBytes(body);
  resource.body = std::make_shared<const std::string>(std::move(body));
  return resource;
}

std::array<double, 3> toArray(const Eigen::Vector3d& v) { return { v.x(), v.y(), v.z() }; }

ViewerMarker poseMarker(const char* type,
                        const Eigen::Isometry3d& pose,
                        const std::string& parent_link,
                        const std::string& name,
                        const std::vector<std::string>& tags)
{
  const Eigen::Quaterniond q(pose.rotation());
  ViewerMarker m;
  m.marker_type = type;
  m.parent_link_name = parent_link;
  m.position = toArray(pose.translation());
  m.quaternion = { q.w(), q.x(), q.y(), q.z() };
  m.name = name;
  m.tags = tags;
  return m;
}
}  // namespace

void ViewerState::setMessageCallback(MessageCallback callback)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  callback_ = std::move(callback);
}

void ViewerState::send(std::string message)
{
  if (callback_)
    callback_(std::make_shared<const std::string>(std::move(message)));
}

void ViewerState::setEnvironment(const tesseract::environment::Environment& env, const std::string& scene_name)
{
  SceneDocuments documents = environmentToDocuments(env, scene_name);
  setScene(std::move(documents.gltf), std::string(documents.glb.begin(), documents.glb.end()));
}

void ViewerState::setScene(std::string gltf, std::string glb)
{
  ViewerResource gltf_resource = makeResource(std::move(gltf));
  ViewerResource glb_resource = makeResource(std::move(glb));

  const std::lock_guard<std::mutex> lock(mutex_);
  scene_gltf_ = std::move(gltf_resource);
  scene_glb_ = std::move(glb_resource);
  send(R"({"command":"refresh_scene"})");
}

void ViewerState::storeTrajectory(std::string trajectory_json)
{
  // The params are already JSON, so the message can be assembled without parsing them again
  std::string message = R"({"command":"joint_trajectory","params":)" + trajectory_json + "}";
  trajectory_json_ = makeResource(std::move(trajectory_json));
  send(std::move(message));
}

void ViewerState::setTrajectoryJson(std::string trajectory_json)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  storeTrajectory(std::move(trajectory_json));
}

void ViewerState::setTrajectory(const tesseract::common::JointTrajectory& trajectory)
{
  setTrajectoryJson(jointTrajectoryToJson(trajectory));
}

void ViewerState::setTrajectoryList(const TrajectoryList& trajectory)
{
  setTrajectoryJson(trajectoryListToJson(trajectory));
}

void ViewerState::setJointPositions(const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_positions)
{
  setTrajectoryJson(jointPositionsToTrajectoryJson(joint_names, joint_positions));
}

void ViewerState::setSceneState(const tesseract::scene_graph::SceneState& state)
{
  setTrajectoryJson(sceneStateToTrajectoryJson(state));
}

std::string ViewerState::appendMarker(ViewerMarker marker)
{
  if (marker.name.empty())
    marker.name = "axes_marker_" + std::to_string(marker_count_);
  ++marker_count_;
  std::string name = marker.name;
  markers_.push_back(std::move(marker));
  return name;
}

void ViewerState::publishMarkers(bool update_now)
{
  std::string json = markersToJson(markers_);
  std::string message;
  if (update_now)
    message = R"({"command":"markers","params":)" + json + "}";
  markers_json_ = makeResource(std::move(json));
  if (update_now)
    send(std::move(message));
}

std::vector<std::string> ViewerState::addMarkers(std::vector<ViewerMarker> markers, bool update_now)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(markers.size());
  for (auto& marker : markers)
    names.push_back(appendMarker(std::move(marker)));
  publishMarkers(update_now);
  return names;
}

std::string ViewerState::addMarker(ViewerMarker marker, bool update_now)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  std::string name = appendMarker(std::move(marker));
  publishMarkers(update_now);
  return name;
}

std::string ViewerState::addAxesMarker(const Eigen::Isometry3d& pose,
                                       double size,
                                       const std::string& parent_link,
                                       const std::string& name,
                                       const std::vector<std::string>& tags,
                                       bool update_now)
{
  ViewerMarker m = poseMarker("axes", pose, parent_link, name, tags);
  m.size = size;
  return addMarker(std::move(m), update_now);
}

std::string ViewerState::addArrowMarker(const Eigen::Vector3d& direction,
                                        const Eigen::Vector3d& origin,
                                        double length,
                                        const std::array<double, 4>& color,
                                        const std::string& parent_link,
                                        const std::string& name,
                                        const std::vector<std::string>& tags,
                                        bool update_now)
{
  ViewerMarker m = poseMarker("arrow", Eigen::Isometry3d::Identity(), parent_link, name, tags);
  m.color = color;
  m.direction = toArray(direction);
  m.origin = toArray(origin);
  m.length = length;
  return addMarker(std::move(m), update_now);
}

std::string ViewerState::addBoxMarker(const Eigen::Isometry3d& pose,
                                      const Eigen::Vector3d& size,
                                      const std::array<double, 4>& color,
                                      const std::string& parent_link,
                                      const std::string& name,
                                      const std::vector<std::string>& tags,
                                      bool update_now)
{
  ViewerMarker m = poseMarker("box", pose, parent_link, name, tags);
  m.color = color;
  m.extents = toArray(size);
  return addMarker(std::move(m), update_now);
}

std::string ViewerState::addSphereMarker(const Eigen::Vector3d& position,
                                         double radius,
                                         const std::array<double, 4>& color,
                                         const std::string& parent_link,
                                         const std::string& name,
                                         const std::vector<std::string>& tags,
                                         bool update_now)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = position;
  ViewerMarker m = poseMarker("sphere", pose, parent_link, name, tags);
  m.color = color;
  m.radius = radius;
  return addMarker(std::move(m), update_now);
}

std::string ViewerState::addCylinderMarker(const Eigen::Isometry3d& pose,
                                           double radius,
                                           double length,
                                           const std::array<double, 4>& color,
                                           const std::string& parent_link,
                                           const std::string& name,
                                           const std::vector<std::string>& tags,
                                           bool update_now)
{
  ViewerMarker m = poseMarker("cylinder", pose, parent_link, name, tags);
  m.color = color;
  m.radius = radius;
  m.length = length;
  return addMarker(std::move(m), update_now);
}

std::string ViewerState::addCapsuleMarker(const Eigen::Isometry3d& pose,
                                          double radius,
                                          double length,
                                          const std::array<double, 4>& color,
                                          const std::string& parent_link,
                                          const std::string& name,
                                          const std::vector<std::string>& tags,
                                          bool update_now)
{
  ViewerMarker m = poseMarker("capsule", pose, parent_link, name, tags);
  m.color = color;
  m.radius = radius;
  m.length = length;
  return addMarker(std::move(m), update_now);
}

std::string ViewerState::addLinesMarker(std::vector<std::array<double, 3>> vertices,
                                        const std::array<double, 4>& color,
                                        double linewidth,
                                        const std::string& parent_link,
                                        const std::string& name,
                                        const std::vector<std::string>& tags,
                                        bool update_now)
{
  ViewerMarker m = poseMarker("lines", Eigen::Isometry3d::Identity(), parent_link, name, tags);
  m.color = color;
  m.vertices = std::move(vertices);
  m.linewidth = linewidth;
  return addMarker(std::move(m), update_now);
}

void ViewerState::plotTrajectoryFrames(const tesseract::common::VectorIsometry3d& frames,
                                       const std::string& parent_link,
                                       const std::array<double, 4>& color,
                                       double linewidth,
                                       bool axes,
                                       double axes_length,
                                       std::vector<std::string> tags,
                                       bool update_now)
{
  tags.push_back("trajectory");

  std::vector<ViewerMarker> markers;
  markers.reserve(1 + (axes ? frames.size() : 0));

  ViewerMarker line = poseMarker("lines", Eigen::Isometry3d::Identity(), parent_link, "", tags);
  line.color = color;
  line.linewidth = linewidth;
  std::vector<std::array<double, 3>> vertices;
  vertices.reserve(frames.size());
  for (const auto& frame : frames)
    vertices.push_back(toArray(frame.translation()));
  line.vertices = std::move(vertices);
  markers.push_back(std::move(line));

  if (axes)
  {
    for (const auto& frame : frames)
    {
      ViewerMarker m = poseMarker("axes", frame, parent_link, "", tags);
      m.size = axes_length;
      markers.push_back(std::move(m));
    }
  }
  addMarkers(std::move(markers), update_now);
}

void ViewerState::clearAllMarkers()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  markers_.clear();
  publishMarkers(true);
}

void ViewerState::clearMarkersByTags(const std::vector<std::string>& tags, bool update_now)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  markers_.erase(std::remove_if(markers_.begin(), markers_.end(), [&](const ViewerMarker& m) { return m.tags == tags; }),
                 markers_.end());
  publishMarkers(update_now);
}

void ViewerState::clearMarkersByName(const std::string& name, bool update_now)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  markers_.erase(std::remove_if(markers_.begin(), markers_.end(), [&](const ViewerMarker& m) { return m.name == name; }),
                 markers_.end());
  publishMarkers(update_now);
}

ViewerResource ViewerState::sceneGltf() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return scene_gltf_;
}

ViewerResource ViewerState::sceneGlb() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return scene_glb_;
}

ViewerResource ViewerState::trajectoryJson() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return trajectory_json_;
}

ViewerResource ViewerState::markersJson() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return markers_json_;
}

}  // namespace tesseract::visualization::threejs
