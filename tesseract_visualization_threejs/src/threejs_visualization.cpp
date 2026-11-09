#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/logging.h>
#include <tesseract/environment/environment.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/visualization/markers/marker.h>
#include <tesseract/visualization/threejs/threejs_visualization.h>
#include <tesseract/visualization/threejs/viewer_server.h>
#include <tesseract/visualization/threejs/viewer_state.h>

namespace tesseract::visualization::threejs
{
namespace
{
constexpr int FIRST_DEFAULT_PORT = 8000;
constexpr int DEFAULT_PORT_ATTEMPTS = 100;

}  // namespace

ThreejsVisualizationOptions ThreejsVisualizationOptions::fromEnvironment()
{
  ThreejsVisualizationOptions options;
  if (const char* host = std::getenv("TESSERACT_VISUALIZATION_THREEJS_HOST"))
    options.host = host;
  if (const char* text = std::getenv("TESSERACT_VISUALIZATION_THREEJS_PORT"))
  {
    std::size_t parsed = 0;
    const int port = std::stoi(text, &parsed);
    if (text[parsed] != '\0' || port < 0 || port > std::numeric_limits<unsigned short>::max())
      throw std::invalid_argument(std::string("Invalid TESSERACT_VISUALIZATION_THREEJS_PORT: ") + text);
    options.port = static_cast<unsigned short>(port);
  }
  return options;
}

namespace
{
constexpr double UNTIMED_TRAJECTORY_DURATION = 5.0;

/** @brief The client animates by time, so a path with no timing is spread evenly, as Ignition does */
tesseract::common::JointTrajectory withUsableTimes(tesseract::common::JointTrajectory trajectory)
{
  const auto& states = trajectory.states;
  const bool untimed = states.size() > 1 && std::all_of(states.begin(), states.end(), [&states](const auto& state) {
                         return state.time == states.front().time;
                       });
  if (untimed)
    for (std::size_t i = 0; i < trajectory.states.size(); ++i)
      trajectory.states[i].time = UNTIMED_TRAJECTORY_DURATION * static_cast<double>(i) / static_cast<double>(states.size() - 1);
  return trajectory;
}

std::vector<std::string> tagsForNamespace(const std::string& ns)
{
  return ns.empty() ? std::vector<std::string>{} : std::vector<std::string>{ ns };
}
}  // namespace

ThreejsVisualization::ThreejsVisualization() : state_(std::make_shared<ViewerState>()) {}

ThreejsVisualization::ThreejsVisualization(ThreejsVisualizationOptions options)
  : options_(std::move(options)), state_(std::make_shared<ViewerState>())
{
}

ThreejsVisualization::~ThreejsVisualization() = default;

unsigned short ThreejsVisualization::port() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return server_ != nullptr ? server_->port() : 0;
}

std::shared_ptr<ViewerState> ThreejsVisualization::state() const { return state_; }

bool ThreejsVisualization::ensureStarted() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (start_attempted_)
    return server_ != nullptr;
  start_attempted_ = true;

  try
  {
    const ThreejsVisualizationOptions options = options_ ? *options_ : ThreejsVisualizationOptions::fromEnvironment();
    const int first_port = options.port ? *options.port : FIRST_DEFAULT_PORT;
    const int attempts = options.port ? 1 : DEFAULT_PORT_ATTEMPTS;
    for (int i = 0; i < attempts && server_ == nullptr; ++i)
    {
      try
      {
        auto server = std::make_unique<ViewerServer>(state_, options.host, static_cast<unsigned short>(first_port + i));
        server->start();
        server_ = std::move(server);
      }
      catch (const std::exception&)
      {
        if (i + 1 == attempts)
          throw;
      }
    }
    TESSERACT_LOG_INFO("tesseract_visualization_threejs: serving on http://{}:{}", options.host, server_->port());
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_ERROR("tesseract_visualization_threejs: could not start the server: {}", e.what());
  }
  return server_ != nullptr;
}

template <class Function>
void ThreejsVisualization::guard(Function&& function)
{
  if (!ensureStarted())
    return;
  try
  {
    function();
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_ERROR("tesseract_visualization_threejs: plotting failed: {}", e.what());
  }
}

std::string ThreejsVisualization::rootLink() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return root_link_;
}

bool ThreejsVisualization::isConnected() const { return ensureStarted() && server_->connectionCount() > 0; }

void ThreejsVisualization::waitForConnection(long seconds) const
{
  if (!ensureStarted())
    return;
  if (seconds > 0)
  {
    server_->waitForConnection(std::chrono::seconds(seconds));
    return;
  }
  while (!server_->waitForConnection(std::chrono::seconds(1)))
  {
  }
}

void ThreejsVisualization::plotEnvironment(const tesseract::environment::Environment& env, std::string /*ns*/)
{
  guard([&] {
    state_->setEnvironment(env);
    const std::lock_guard<std::mutex> lock(mutex_);
    root_link_ = env.getRootLinkId().name();
  });
}

void ThreejsVisualization::plotEnvironmentState(const tesseract::scene_graph::SceneState& state, std::string /*ns*/)
{
  if (state.joints.empty())
    return;
  guard([&] { state_->setSceneState(state); });
}

void ThreejsVisualization::plotTrajectory(const tesseract::common::JointTrajectory& traj,
                                          const tesseract::scene_graph::StateSolver& /*state_solver*/,
                                          std::string /*ns*/)
{
  guard([&] { state_->setTrajectory(withUsableTimes(traj)); });
}

void ThreejsVisualization::plotMarker(const tesseract::visualization::Marker& marker, std::string ns)
{
  guard([&] { state_->addMarkers(convertMarker(marker, rootLink(), tagsForNamespace(ns))); });
}

void ThreejsVisualization::plotMarkers(const std::vector<std::shared_ptr<tesseract::visualization::Marker>>& markers,
                                       std::string ns)
{
  guard([&] {
    const std::string root = rootLink();
    const std::vector<std::string> tags = tagsForNamespace(ns);
    std::vector<ViewerMarker> converted;
    for (const auto& marker : markers)
    {
      auto part = convertMarker(*marker, root, tags);
      converted.insert(converted.end(), std::make_move_iterator(part.begin()), std::make_move_iterator(part.end()));
    }
    state_->addMarkers(std::move(converted));
  });
}

void ThreejsVisualization::clear(std::string ns)
{
  guard([&] {
    if (ns.empty())
      state_->clearAllMarkers();
    else
      state_->clearMarkersByTags(tagsForNamespace(ns));
  });
}

void ThreejsVisualization::waitForInput(std::string message)
{
  std::cout << message << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

}  // namespace tesseract::visualization::threejs
