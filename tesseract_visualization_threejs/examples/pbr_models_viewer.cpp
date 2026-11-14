// Displays glTF 2.0 sample models that use every PBR texture slot, on turntables, with the alpha mode, emissive
// strength, metal-roughness and normal map test boards behind them.
//
// The models are not part of this repository. Download them first:
//   python3 examples/download_pbr_models.py pbr_models
//   tesseract_visualization_threejs_pbr_models_viewer pbr_models
// Then open the URL it prints. ?lighting=environment adds the environment map that PBR materials need to look right.

#include <cmath>
#include <filesystem>
#include <iostream>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/threejs/threejs_visualization.h>

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: tesseract_visualization_threejs_pbr_models_viewer <models directory or pbr_models.urdf>\n"
                 "Download the models with examples/download_pbr_models.py\n";
    return 2;
  }

  std::filesystem::path urdf(argv[1]);
  if (std::filesystem::is_directory(urdf))
    urdf /= "pbr_models.urdf";
  if (!std::filesystem::exists(urdf))
  {
    std::cerr << urdf << " does not exist. Download the models with examples/download_pbr_models.py\n";
    return 1;
  }

  tesseract::environment::Environment env;
  if (!env.init(urdf, std::make_shared<tesseract::common::GeneralResourceLocator>()))
  {
    std::cerr << "Failed to load " << urdf << "\n";
    return 1;
  }

  tesseract::visualization::threejs::ThreejsVisualization viewer;
  viewer.plotEnvironment(env);

  // Turn every turntable once every 12 seconds. The client plays the trajectory in a loop.
  const auto ids = env.getStateSolver()->getActiveJointIds();
  tesseract::common::JointTrajectory trajectory;
  constexpr int STEPS = 120;
  for (int i = 0; i <= STEPS; ++i)
  {
    const double phase = static_cast<double>(i) / STEPS;
    tesseract::common::JointState state(
        ids, Eigen::VectorXd::Constant(static_cast<Eigen::Index>(ids.size()), 2.0 * EIGEN_PI * phase));
    state.time = phase * 12.0;
    trajectory.push_back(state);
  }
  viewer.plotTrajectory(trajectory, *env.getStateSolver());

  std::cout << "Open http://127.0.0.1:" << viewer.port() << "/?lighting=environment\n"
            << "Model licenses: " << (urdf.parent_path() / "ATTRIBUTION.md").string() << std::endl;
  viewer.waitForInput("Press enter to quit");
  return 0;
}
