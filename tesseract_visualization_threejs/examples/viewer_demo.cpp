// Serves a robot, a trajectory and markers. Open http://localhost:8000 in a browser.
// usage: tesseract_visualization_threejs_viewer_demo <urdf path> <srdf path> [host] [port]
// TESSERACT_RESOURCE_PATH must let package:// URLs in the URDF resolve.

#include <cmath>
#include <filesystem>
#include <iostream>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/visualization/threejs/viewer_server.h>
#include <tesseract/visualization/threejs/viewer_state.h>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "usage: tesseract_visualization_threejs_viewer_demo <urdf path> <srdf path> [host] [port]\n";
    return 2;
  }

  auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  tesseract::environment::Environment env;
  if (!env.init(std::filesystem::path(argv[1]), std::filesystem::path(argv[2]), locator))
  {
    std::cerr << "Failed to load the environment\n";
    return 1;
  }

  auto state = std::make_shared<tesseract::visualization::threejs::ViewerState>();
  state->setEnvironment(env);

  // Sweep every active joint one radian and back
  const auto joints = env.getActiveJointIds();
  tesseract::visualization::threejs::TrajectoryList trajectory;
  for (const auto& id : joints)
    trajectory.joint_names.push_back(id.name());
  for (int i = 0; i <= 40; ++i)
  {
    const double t = i / 40.0;
    std::vector<double> row(joints.size(), 1.0 - std::abs(2.0 * t - 1.0));
    row.push_back(t * 10.0);
    trajectory.trajectory.push_back(std::move(row));
  }
  state->setTrajectoryList(trajectory);

  const std::string root = env.getRootLinkId().name();
  state->addAxesMarker(Eigen::Isometry3d::Identity(), 0.3, root, "origin");
  state->addSphereMarker(Eigen::Vector3d(0.8, 0, 0.5), 0.05, { 1, 0, 0, 1 }, root, "target");

  const std::string host = argc > 3 ? argv[3] : "127.0.0.1";
  const auto port = static_cast<unsigned short>(argc > 4 ? std::stoi(argv[4]) : 8000);
  tesseract::visualization::threejs::ViewerServer server(state, host, port);
  server.start();
  std::cout << "Serving on http://" << host << ":" << server.port() << "\nPress enter to quit" << std::endl;
  std::cin.get();
  server.stop();
  return 0;
}
