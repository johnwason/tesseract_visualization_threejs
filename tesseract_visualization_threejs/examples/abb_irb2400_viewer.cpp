// Based on tesseract_viewer_python/examples/abb_irb2400_viewer.py
// Displays an ABB IRB 2400, animates a joint trajectory, and draws the tool's path with axes at each
// waypoint. TESSERACT_RESOURCE_PATH must resolve package://tesseract.
//
// The Python example plans its trajectory with OMPL and TrajOpt. Those are in tesseract_planning,
// which this project does not depend on, so this interpolates between joint positions instead.
// Open the URL it logs in a browser, usually http://127.0.0.1:8000

#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/threejs/conversions.h>
#include <tesseract/visualization/threejs/threejs_visualization.h>
#include <tesseract/visualization/threejs/viewer_state.h>

using tesseract::common::JointId;

static std::filesystem::path locate(const tesseract::common::ResourceLocator& locator, const std::string& url)
{
  const auto resource = locator.locateResource(url);
  if (resource == nullptr)
    throw std::runtime_error("Cannot find " + url + ". Check TESSERACT_RESOURCE_PATH.");
  return resource->getFilePath();
}

int main()
{
  auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  tesseract::environment::Environment env;
  if (!env.init(locate(*locator, "package://tesseract/support/urdf/abb_irb2400.urdf"),
                locate(*locator, "package://tesseract/support/urdf/abb_irb2400.srdf"),
                locator))
  {
    std::cerr << "Failed to load the environment\n";
    return 1;
  }

  tesseract::visualization::threejs::ThreejsVisualization viewer;
  viewer.plotEnvironment(env);

  std::vector<JointId> joints;
  for (int i = 1; i <= 6; ++i)
    joints.emplace_back("joint_" + std::to_string(i));

  Eigen::VectorXd goal(6);
  goal << 1, -0.2, 0.01, 0.3, -0.5, 1;

  tesseract::common::JointTrajectory trajectory;
  const int steps = 50;
  for (int i = 0; i <= steps; ++i)
  {
    tesseract::common::JointState state(joints, goal * (static_cast<double>(i) / steps));
    state.time = 0.1 * i;
    trajectory.states.push_back(state);
  }
  viewer.plotTrajectory(trajectory, *env.getStateSolver());

  namespace threejs = tesseract::visualization::threejs;
  const auto frames = threejs::trajectoryToFrames(env, "manipulator", "tool0", threejs::jointTrajectoryToList(trajectory));
  viewer.state()->plotTrajectoryFrames(frames, env.getRootLinkId().name(), { 0, 1, 0, 1 }, 0.001, true, 0.05);

  viewer.waitForInput("press enter");
  return 0;
}
