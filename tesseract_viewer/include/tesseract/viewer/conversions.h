/**
 * @file conversions.h
 * @brief Layer 1: pure conversions from tesseract types to the viewer's wire formats. No networking.
 *
 * Ported from tesseract_viewer_python. PORTING_NOTES.md lists where the port deliberately differs.
 */
#ifndef TESSERACT_VIEWER_CONVERSIONS_H
#define TESSERACT_VIEWER_CONVERSIONS_H

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>

#include <tesseract/common/eigen_types.h>
#include <tesseract/common/fwd.h>
#include <tesseract/environment/fwd.h>
#include <tesseract/scene_graph/fwd.h>
#include <tesseract/visualization/fwd.h>

namespace tesseract::viewer
{
/**
 * @brief Convert an environment to a self-contained glTF 2.0 document, with the buffer as a base64 URI
 * @param env The environment
 * @param scene_name The glTF scene name. Omitted when empty
 * @param trajectory If set, appended to the document as a glTF animation
 * @throws std::invalid_argument if the trajectory names an unknown or non-animatable joint
 */
std::string environmentToGltf(const tesseract::environment::Environment& env,
                              const std::string& scene_name = "",
                              const tesseract::common::JointTrajectory* trajectory = nullptr);

/** @brief Convert an environment to a binary glTF (GLB) container */
std::vector<std::uint8_t> environmentToGlb(const tesseract::environment::Environment& env,
                                           const std::string& scene_name = "");

/** @brief Joint names and rows of [q0, ..., qn, t], the client's trajectory layout */
struct TrajectoryList
{
  std::vector<std::string> joint_names;
  std::vector<std::vector<double>> trajectory;
};

/** @throws std::invalid_argument if empty, or if a state's joints differ from the first state's */
TrajectoryList jointTrajectoryToList(const tesseract::common::JointTrajectory& trajectory);

/** @throws std::invalid_argument if a row does not hold one value per joint plus the time */
std::string trajectoryListToJson(const TrajectoryList& trajectory, bool use_time = true, double loop_time = 20);

std::string jointTrajectoryToJson(const tesseract::common::JointTrajectory& trajectory,
                                  bool use_time = true,
                                  double loop_time = 20);

/**
 * @brief A static joint state, as the two-point trajectory the client animates forever
 * @throws std::invalid_argument if names are empty or sizes differ
 */
std::string jointPositionsToTrajectoryJson(const std::vector<std::string>& joint_names,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_positions);

/** @brief A SceneState's joint values as a static trajectory. Joints are sorted by name for a stable ETag */
std::string sceneStateToTrajectoryJson(const tesseract::scene_graph::SceneState& state);

/**
 * @brief Forward kinematics of a tip link along a trajectory
 * @param env The environment
 * @param group_name The joint group
 * @param tip_link The link whose pose is returned
 * @param trajectory The trajectory; joints are matched by name, so its order need not be the group's
 */
tesseract::common::VectorIsometry3d trajectoryToFrames(const tesseract::environment::Environment& env,
                                                       const std::string& group_name,
                                                       const std::string& tip_link,
                                                       const TrajectoryList& trajectory);

/**
 * @brief One entry of the client's markers document
 * @details marker_type is one of "axes", "arrow", "box", "sphere", "cylinder", "cone", "capsule" or
 * "lines". The optional members are written only when set.
 */
struct ViewerMarker
{
  std::string marker_type;
  std::string parent_link_name{ "world" };
  std::array<double, 3> position{ 0, 0, 0 };
  /** @brief w, x, y, z */
  std::array<double, 4> quaternion{ 1, 0, 0, 0 };
  /** @brief Generated when empty, as axes_marker_N, by ViewerState */
  std::string name;
  std::array<double, 4> color{ 1, 1, 1, 1 };
  std::vector<std::string> tags;

  std::optional<double> size;                   ///< axes
  std::optional<std::array<double, 3>> extents; ///< box; written as "size"
  std::optional<std::array<double, 3>> direction;
  std::optional<std::array<double, 3>> origin;
  std::optional<double> length;
  std::optional<double> radius;
  std::optional<std::vector<std::array<double, 3>>> vertices;
  std::optional<double> linewidth;
};

/** @brief Serialize to the client's {"markers": [...]} document */
std::string markersToJson(const std::vector<ViewerMarker>& markers);

/**
 * @brief Map a tesseract marker onto the client's marker types
 * @details Supported: AXIS, ARROW, GEOMETRY (box, sphere, cylinder, cone, capsule), TOOLPATH and
 * CONTACT_RESULTS. One tesseract marker can produce several client markers. Anything else logs a
 * warning and returns nothing, because the client cannot show it.
 * @param marker The tesseract marker
 * @param default_parent_link Used when the marker has no parent link, usually the root link
 * @param tags Tags for every generated marker
 */
std::vector<ViewerMarker> convertMarker(const tesseract::visualization::Marker& marker,
                                        const std::string& default_parent_link,
                                        const std::vector<std::string>& tags = {});

/** @brief SHA-256 of the data as lowercase hex. Used for ETags */
std::string hashBytes(std::string_view data);

/** @brief Contents of a compiled-in client file: "index.html" or "app.js". Empty if unknown */
std::string_view staticFile(std::string_view name);

}  // namespace tesseract::viewer

#endif  // TESSERACT_VIEWER_CONVERSIONS_H
