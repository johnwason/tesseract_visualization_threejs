// Port of util.py

#include <algorithm>
#include <stdexcept>

#include <tesseract/common/joint_state.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/viewer/conversions.h>

#include "json_util.h"

namespace tesseract::viewer
{
TrajectoryList jointTrajectoryToList(const tesseract::common::JointTrajectory& trajectory)
{
  if (trajectory.states.empty())
    throw std::invalid_argument("Trajectory is empty");

  TrajectoryList out;
  const auto& first_ids = trajectory.states.front().joint_ids;
  out.joint_names.reserve(first_ids.size());
  for (const auto& id : first_ids)
    out.joint_names.push_back(id.name());

  out.trajectory.reserve(trajectory.states.size());
  for (const auto& state : trajectory.states)
  {
    if (state.joint_ids != first_ids)
      throw std::invalid_argument("Trajectory states have different joints");
    if (state.position.size() != static_cast<Eigen::Index>(first_ids.size()))
      throw std::invalid_argument("Trajectory state position size does not match its joints");

    std::vector<double> row(state.position.data(), state.position.data() + state.position.size());
    row.push_back(state.time);
    out.trajectory.push_back(std::move(row));
  }
  return out;
}

std::string trajectoryListToJson(const TrajectoryList& trajectory, bool use_time, double loop_time)
{
  Json::Value root;
  root["use_time"] = use_time;
  root["loop_time"] = loop_time;
  root["joint_names"] = detail::toJsonArray(trajectory.joint_names);
  root["trajectory"] = Json::Value(Json::arrayValue);
  for (const auto& row : trajectory.trajectory)
  {
    if (row.size() != trajectory.joint_names.size() + 1)
      throw std::invalid_argument("Trajectory row must hold one value per joint plus the time");
    root["trajectory"].append(detail::toJsonArray(row));
  }
  return detail::writeJson(root);
}

std::string jointTrajectoryToJson(const tesseract::common::JointTrajectory& trajectory, bool use_time, double loop_time)
{
  return trajectoryListToJson(jointTrajectoryToList(trajectory), use_time, loop_time);
}

std::string jointPositionsToTrajectoryJson(const std::vector<std::string>& joint_names,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_positions)
{
  if (joint_names.empty())
    throw std::invalid_argument("Joint names must not be empty");
  if (joint_positions.size() != static_cast<Eigen::Index>(joint_names.size()))
    throw std::invalid_argument("Joint names and positions differ in size");

  TrajectoryList list;
  list.joint_names = joint_names;
  std::vector<double> row(joint_positions.data(), joint_positions.data() + joint_positions.size());
  row.push_back(0.0);
  list.trajectory.push_back(row);
  row.back() = 1e6;
  list.trajectory.push_back(std::move(row));
  return trajectoryListToJson(list, true, 10000);
}

std::string sceneStateToTrajectoryJson(const tesseract::scene_graph::SceneState& state)
{
  std::vector<std::pair<std::string, double>> joints;
  joints.reserve(state.joints.size());
  for (const auto& [id, value] : state.joints)
    joints.emplace_back(id.name(), value);
  std::sort(joints.begin(), joints.end());

  std::vector<std::string> names;
  Eigen::VectorXd positions(static_cast<Eigen::Index>(joints.size()));
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    names.push_back(joints[i].first);
    positions[static_cast<Eigen::Index>(i)] = joints[i].second;
  }
  return jointPositionsToTrajectoryJson(names, positions);
}

tesseract::common::VectorIsometry3d trajectoryToFrames(const tesseract::environment::Environment& env,
                                                       const std::string& group_name,
                                                       const std::string& tip_link,
                                                       const TrajectoryList& trajectory)
{
  const auto group = env.getJointGroup(group_name);
  if (group == nullptr)
    throw std::invalid_argument("Unknown joint group: " + group_name);

  const auto& group_joints = group->getJointIds();
  std::vector<std::size_t> columns;
  columns.reserve(group_joints.size());
  for (const auto& id : group_joints)
  {
    const auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), id.name());
    if (it == trajectory.joint_names.end())
      throw std::invalid_argument("Trajectory is missing group joint: " + id.name());
    columns.push_back(static_cast<std::size_t>(std::distance(trajectory.joint_names.begin(), it)));
  }

  const tesseract::common::LinkId tip(tip_link);
  tesseract::common::VectorIsometry3d frames;
  frames.reserve(trajectory.trajectory.size());
  Eigen::VectorXd q(static_cast<Eigen::Index>(columns.size()));
  for (const auto& row : trajectory.trajectory)
  {
    if (row.size() != trajectory.joint_names.size() + 1)
      throw std::invalid_argument("Trajectory row must hold one value per joint plus the time");
    for (std::size_t i = 0; i < columns.size(); ++i)
      q[static_cast<Eigen::Index>(i)] = row[columns[i]];

    const auto transforms = group->calcFwdKin(q);
    const auto it = transforms.find(tip);
    if (it == transforms.end())
      throw std::invalid_argument("Link is not in the joint group: " + tip_link);
    frames.push_back(it->second);
  }
  return frames;
}

}  // namespace tesseract::viewer
