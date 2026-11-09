/**
 * @file threejs_visualization.h
 * @brief A tesseract::visualization::Visualization that serves the three.js web viewer
 */
#ifndef TESSERACT_VISUALIZATION_THREEJS_THREEJS_VISUALIZATION_H
#define TESSERACT_VISUALIZATION_THREEJS_THREEJS_VISUALIZATION_H

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <tesseract/visualization/visualization.h>

namespace tesseract::visualization::threejs
{
class ViewerServer;
class ViewerState;

struct ThreejsVisualizationOptions
{
  /** @brief Address to bind. Localhost by default because the viewer has no authentication */
  std::string host{ "127.0.0.1" };
  /** @brief Port to bind, 0 for any free port. If unset, the first free port from 8000 is used */
  std::optional<unsigned short> port;
};

/**
 * @brief Owns one independent web server and scene per object
 * @details The server starts on first use, and its URL is logged. A failure to start or plot is
 * logged rather than thrown, so that the viewer cannot abort an application's planning.
 */
class ThreejsVisualization : public tesseract::visualization::Visualization
{
public:
  /** @brief Options come from TESSERACT_VISUALIZATION_THREEJS_HOST and _PORT when the server starts */
  ThreejsVisualization();
  explicit ThreejsVisualization(ThreejsVisualizationOptions options);
  ~ThreejsVisualization() override;
  ThreejsVisualization(const ThreejsVisualization&) = delete;
  ThreejsVisualization& operator=(const ThreejsVisualization&) = delete;
  ThreejsVisualization(ThreejsVisualization&&) = delete;
  ThreejsVisualization& operator=(ThreejsVisualization&&) = delete;

  /** @brief The bound port, or 0 until the server has started */
  unsigned short port() const;

  /** @brief This object's scene, for serving or modifying it directly */
  std::shared_ptr<ViewerState> state() const;

  bool isConnected() const override;
  void waitForConnection(long seconds = 0) const override;
  void plotEnvironment(const tesseract::environment::Environment& env, std::string ns = "") override;
  void plotEnvironmentState(const tesseract::scene_graph::SceneState& state, std::string ns = "") override;
  void plotTrajectory(const tesseract::common::JointTrajectory& traj,
                      const tesseract::scene_graph::StateSolver& state_solver,
                      std::string ns = "") override;
  void plotMarker(const tesseract::visualization::Marker& marker, std::string ns = "") override;
  void plotMarkers(const std::vector<std::shared_ptr<tesseract::visualization::Marker>>& markers,
                   std::string ns = "") override;
  void clear(std::string ns = "") override;
  void waitForInput(std::string message = "Hit enter key to continue!") override;

private:
  bool ensureStarted() const;
  template <class Function>
  void guard(Function&& function);
  std::string rootLink() const;

  mutable std::mutex mutex_;
  std::optional<ThreejsVisualizationOptions> options_;
  std::shared_ptr<ViewerState> state_;
  mutable std::unique_ptr<ViewerServer> server_;
  std::string root_link_{ "world" };
  mutable bool start_attempted_{ false };
};

}  // namespace tesseract::visualization::threejs

#endif  // TESSERACT_VISUALIZATION_THREEJS_THREEJS_VISUALIZATION_H
