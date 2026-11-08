/**
 * @file viewer_state.h
 * @brief Layer 2: current scene, trajectory and markers, and the websocket messages. No transport.
 *
 * tesseract_robotraconteur and similar consumers can use this directly and serve it however they like.
 */
#ifndef TESSERACT_VISUALIZATION_THREEJS_VIEWER_STATE_H
#define TESSERACT_VISUALIZATION_THREEJS_VIEWER_STATE_H

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <tesseract/visualization/threejs/conversions.h>

namespace tesseract::visualization::threejs
{
/** @brief A served document. body is null until the document has been set; etag is then "None" */
struct ViewerResource
{
  std::shared_ptr<const std::string> body;
  std::string etag{ "None" };
};

/** @brief Thread-safe holder of everything the client fetches, and the source of its update messages */
class ViewerState
{
public:
  /**
   * @brief Receives each websocket message, a JSON {"command": ..., "params": ...} text
   * @details Called with the state's lock held so that messages keep their order. It must return
   * quickly and must not call back into this object.
   */
  using MessageCallback = std::function<void(std::shared_ptr<const std::string>)>;

  void setMessageCallback(MessageCallback callback);

  /** @brief Convert the environment and send refresh_scene */
  void setEnvironment(const tesseract::environment::Environment& env, const std::string& scene_name = "");
  /** @brief Store an already converted scene and send refresh_scene */
  void setScene(std::string gltf, std::string glb);

  void setTrajectory(const tesseract::common::JointTrajectory& trajectory);
  void setTrajectoryList(const TrajectoryList& trajectory);
  /** @brief Store a trajectory document and send joint_trajectory */
  void setTrajectoryJson(std::string trajectory_json);
  void setJointPositions(const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_positions);
  void setSceneState(const tesseract::scene_graph::SceneState& state);

  /**
   * @brief Append markers
   * @param markers Markers to add. Empty names are generated
   * @param update_now Send the markers message. Pass false when adding in several steps
   * @return The final marker names
   */
  std::vector<std::string> addMarkers(std::vector<ViewerMarker> markers, bool update_now = true);
  std::string addMarker(ViewerMarker marker, bool update_now = true);

  std::string addAxesMarker(const Eigen::Isometry3d& pose,
                            double size = 0.15,
                            const std::string& parent_link = "world",
                            const std::string& name = "",
                            const std::vector<std::string>& tags = {},
                            bool update_now = true);
  std::string addArrowMarker(const Eigen::Vector3d& direction,
                             const Eigen::Vector3d& origin,
                             double length,
                             const std::array<double, 4>& color = { 1, 1, 1, 1 },
                             const std::string& parent_link = "world",
                             const std::string& name = "",
                             const std::vector<std::string>& tags = {},
                             bool update_now = true);
  std::string addBoxMarker(const Eigen::Isometry3d& pose,
                           const Eigen::Vector3d& size,
                           const std::array<double, 4>& color = { 1, 1, 1, 1 },
                           const std::string& parent_link = "world",
                           const std::string& name = "",
                           const std::vector<std::string>& tags = {},
                           bool update_now = true);
  std::string addSphereMarker(const Eigen::Vector3d& position,
                              double radius,
                              const std::array<double, 4>& color = { 1, 1, 1, 1 },
                              const std::string& parent_link = "world",
                              const std::string& name = "",
                              const std::vector<std::string>& tags = {},
                              bool update_now = true);
  /** @brief The client draws cylinders along the pose's Y axis, as in the Python API */
  std::string addCylinderMarker(const Eigen::Isometry3d& pose,
                                double radius,
                                double length,
                                const std::array<double, 4>& color = { 1, 1, 1, 1 },
                                const std::string& parent_link = "world",
                                const std::string& name = "",
                                const std::vector<std::string>& tags = {},
                                bool update_now = true);
  /** @brief Stored and sent, but the current client only logs that capsules are not implemented */
  std::string addCapsuleMarker(const Eigen::Isometry3d& pose,
                               double radius,
                               double length,
                               const std::array<double, 4>& color = { 1, 1, 1, 1 },
                               const std::string& parent_link = "world",
                               const std::string& name = "",
                               const std::vector<std::string>& tags = {},
                               bool update_now = true);
  std::string addLinesMarker(std::vector<std::array<double, 3>> vertices,
                             const std::array<double, 4>& color = { 1, 1, 1, 1 },
                             double linewidth = 1.0,
                             const std::string& parent_link = "world",
                             const std::string& name = "",
                             const std::vector<std::string>& tags = {},
                             bool update_now = true);

  /**
   * @brief Draw a tool path: a line through the frames and, optionally, axes at each frame
   * @details Every marker is tagged "trajectory" in addition to tags.
   */
  void plotTrajectoryFrames(const tesseract::common::VectorIsometry3d& frames,
                            const std::string& parent_link,
                            const std::array<double, 4>& color = { 0, 1, 0, 1 },
                            double linewidth = 0.001,
                            bool axes = true,
                            double axes_length = 0.1,
                            std::vector<std::string> tags = {},
                            bool update_now = true);

  void clearAllMarkers();
  /** @brief Remove markers whose tag list equals tags exactly, as the Python viewer did */
  void clearMarkersByTags(const std::vector<std::string>& tags, bool update_now = true);
  void clearMarkersByName(const std::string& name, bool update_now = true);

  ViewerResource sceneGltf() const;
  ViewerResource sceneGlb() const;
  ViewerResource trajectoryJson() const;
  ViewerResource markersJson() const;

private:
  void send(std::string message);
  void storeTrajectory(std::string trajectory_json);
  void publishMarkers(bool update_now);
  std::string appendMarker(ViewerMarker marker);

  mutable std::mutex mutex_;
  MessageCallback callback_;
  ViewerResource scene_gltf_;
  ViewerResource scene_glb_;
  ViewerResource trajectory_json_;
  ViewerResource markers_json_;
  std::vector<ViewerMarker> markers_;
  std::size_t marker_count_{ 1 };
};

}  // namespace tesseract::visualization::threejs

#endif  // TESSERACT_VISUALIZATION_THREEJS_VIEWER_STATE_H
