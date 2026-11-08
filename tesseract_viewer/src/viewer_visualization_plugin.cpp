/**
 * @file viewer_visualization_plugin.cpp
 * @brief tesseract::visualization::Visualization plugin that serves the web viewer
 *
 * Load it by setting
 *   TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<prefix>/lib
 *   TESSERACT_VISUALIZATION_PLUGINS=tesseract_viewer_visualization_plugin
 * The server binds TESSERACT_VIEWER_HOST (default 127.0.0.1) and TESSERACT_VIEWER_PORT (default 8000).
 */
#include <boost_plugin_loader/macros.h>

#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/logging.h>
#include <tesseract/environment/environment.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/viewer/viewer_server.h>
#include <tesseract/viewer/viewer_state.h>
#include <tesseract/viewer/viewer_visualization_anchor.h>
#include <tesseract/visualization/markers/marker.h>
#include <tesseract/visualization/visualization.h>

namespace tesseract::viewer
{
namespace
{
/** @brief One server per process, whichever exported symbol the application loaded */
class ViewerRuntime
{
public:
  static ViewerRuntime& instance()
  {
    static ViewerRuntime runtime;
    return runtime;
  }

  /** @brief Start the server on first use. @return false if it could not be started */
  bool ensureStarted()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (start_attempted_)
      return started_;
    start_attempted_ = true;

    const char* host_env = std::getenv("TESSERACT_VIEWER_HOST");
    const char* port_env = std::getenv("TESSERACT_VIEWER_PORT");
    const std::string host = host_env != nullptr ? host_env : "127.0.0.1";
    try
    {
      const int port = port_env != nullptr ? std::stoi(port_env) : 8000;
      server_ = std::make_unique<ViewerServer>(state_, host, static_cast<unsigned short>(port));
      server_->start();
      started_ = true;
      TESSERACT_LOG_INFO("tesseract_viewer: serving on http://{}:{}", host, server_->port());
    }
    catch (const std::exception& e)
    {
      // Planning should carry on when the viewer is unavailable, for example if the port is taken
      TESSERACT_LOG_ERROR("tesseract_viewer: could not start the server on {}: {}", host, e.what());
      server_.reset();
    }
    return started_;
  }

  const std::shared_ptr<ViewerState>& state() const { return state_; }
  ViewerServer* server() const { return server_.get(); }

  void setRootLink(const std::string& root)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    root_link_ = root;
  }

  std::string rootLink() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return root_link_;
  }

private:
  ViewerRuntime() = default;

  mutable std::mutex mutex_;
  std::shared_ptr<ViewerState> state_{ std::make_shared<ViewerState>() };
  std::unique_ptr<ViewerServer> server_;
  std::string root_link_{ "world" };
  bool start_attempted_{ false };
  bool started_{ false };
};

std::vector<std::string> tagsForNamespace(const std::string& ns)
{
  return ns.empty() ? std::vector<std::string>{} : std::vector<std::string>{ ns };
}
}  // namespace

class ViewerVisualization : public tesseract::visualization::Visualization
{
public:
  bool isConnected() const override
  {
    auto& runtime = ViewerRuntime::instance();
    return runtime.ensureStarted() && runtime.server()->connectionCount() > 0;
  }

  void waitForConnection(long seconds = 0) const override
  {
    auto& runtime = ViewerRuntime::instance();
    if (!runtime.ensureStarted())
      return;
    if (seconds > 0)
    {
      runtime.server()->waitForConnection(std::chrono::seconds(seconds));
      return;
    }
    while (!runtime.server()->waitForConnection(std::chrono::seconds(1)))
    {
    }
  }

  void plotEnvironment(const tesseract::environment::Environment& env, std::string /*ns*/ = "") override
  {
    guard([&](ViewerRuntime& runtime) {
      runtime.state()->setEnvironment(env);
      runtime.setRootLink(env.getRootLinkId().name());
    });
  }

  void plotEnvironmentState(const tesseract::scene_graph::SceneState& state, std::string /*ns*/ = "") override
  {
    if (state.joints.empty())
      return;
    guard([&](ViewerRuntime& runtime) { runtime.state()->setSceneState(state); });
  }

  void plotTrajectory(const tesseract::common::JointTrajectory& traj,
                      const tesseract::scene_graph::StateSolver& /*state_solver*/,
                      std::string /*ns*/ = "") override
  {
    guard([&](ViewerRuntime& runtime) { runtime.state()->setTrajectory(traj); });
  }

  void plotMarker(const tesseract::visualization::Marker& marker, std::string ns = "") override
  {
    guard([&](ViewerRuntime& runtime) {
      runtime.state()->addMarkers(convertMarker(marker, runtime.rootLink(), tagsForNamespace(ns)));
    });
  }

  void plotMarkers(const std::vector<std::shared_ptr<tesseract::visualization::Marker>>& markers,
                   std::string ns = "") override
  {
    guard([&](ViewerRuntime& runtime) {
      const std::string root = runtime.rootLink();
      const std::vector<std::string> tags = tagsForNamespace(ns);
      std::vector<ViewerMarker> converted;
      for (const auto& marker : markers)
      {
        auto part = convertMarker(*marker, root, tags);
        converted.insert(converted.end(), std::make_move_iterator(part.begin()), std::make_move_iterator(part.end()));
      }
      runtime.state()->addMarkers(std::move(converted));
    });
  }

  void clear(std::string ns = "") override
  {
    guard([&](ViewerRuntime& runtime) {
      if (ns.empty())
        runtime.state()->clearAllMarkers();
      else
        runtime.state()->clearMarkersByTags(tagsForNamespace(ns));
    });
  }

  void waitForInput(std::string message = "Hit enter key to continue!") override
  {
    std::cout << message << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

private:
  /** @brief A plotting failure is logged, so that it cannot abort the application's planning */
  template <class Function>
  static void guard(Function&& function)
  {
    auto& runtime = ViewerRuntime::instance();
    if (!runtime.ensureStarted())
      return;
    try
    {
      function(runtime);
    }
    catch (const std::exception& e)
    {
      TESSERACT_LOG_ERROR("tesseract_viewer: plotting failed: {}", e.what());
    }
  }
};

PLUGIN_ANCHOR_IMPL(ViewerVisualizationAnchor)

}  // namespace tesseract::viewer

// TESSERACT_ADD_VISUALIZATION_PLUGIN expands to a macro that does not exist at the pinned tesseract
// commit, so the underlying boost_plugin_loader macro is used directly.
EXPORT_CLASS_SECTIONED(tesseract::viewer::ViewerVisualization, TesseractViewerVisualizationPlugin, Plotter)

// VisualizationLoader::get() with no name asks for the Ignition symbol. Exporting it too means that
// existing callers of get(), such as the tesseract_planning examples, get the viewer unchanged.
EXPORT_CLASS_SECTIONED(tesseract::viewer::ViewerVisualization, TesseractIgnitionVisualizationPlugin, Plotter)
