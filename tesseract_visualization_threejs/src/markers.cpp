#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <tesseract/collision/types.h>
#include <tesseract/common/logging.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/visualization/threejs/conversions.h>
#include <tesseract/visualization/markers/arrow_marker.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/markers/contact_results_marker.h>
#include <tesseract/visualization/markers/geometry_marker.h>
#include <tesseract/visualization/markers/marker.h>
#include <tesseract/visualization/markers/toolpath_marker.h>

#include "json_util.h"

namespace tesseract::visualization::threejs
{
namespace
{
using detail::toJsonArray;
namespace vis = tesseract::visualization;

std::array<double, 3> toArray(const Eigen::Vector3d& v) { return { v.x(), v.y(), v.z() }; }

std::array<double, 4> toWxyz(const Eigen::Isometry3d& pose)
{
  const Eigen::Quaterniond q(pose.rotation());
  return { q.w(), q.x(), q.y(), q.z() };
}

ViewerMarker baseMarker(const char* type, const std::string& parent, const std::vector<std::string>& tags)
{
  ViewerMarker m;
  m.marker_type = type;
  m.parent_link_name = parent;
  m.tags = tags;
  return m;
}

ViewerMarker arrowMarker(const Eigen::Vector3d& origin,
                         const Eigen::Vector3d& direction,
                         double length,
                         const std::array<double, 4>& color,
                         const std::string& parent,
                         const std::vector<std::string>& tags)
{
  ViewerMarker m = baseMarker("arrow", parent, tags);
  m.color = color;
  m.origin = toArray(origin);
  m.direction = toArray(direction);
  m.length = length;
  return m;
}

void appendArrowBetween(std::vector<ViewerMarker>& out,
                        const Eigen::Vector3d& from,
                        const Eigen::Vector3d& to,
                        const std::array<double, 4>& color,
                        const std::string& parent,
                        const std::vector<std::string>& tags)
{
  const Eigen::Vector3d delta = to - from;
  const double length = delta.norm();
  if (length > 1e-9)
    out.push_back(arrowMarker(from, delta / length, length, color, parent, tags));
}

void appendGeometry(std::vector<ViewerMarker>& out,
                    const vis::GeometryMarker& marker,
                    const std::string& parent,
                    const std::vector<std::string>& tags)
{
  namespace g = tesseract::geometry;
  const g::Geometry* geometry = marker.geom.get();
  ViewerMarker m;

  // three.js builds cylinders and cones along +Y and tesseract along +Z. +90 degrees about X maps Y onto Z.
  const Eigen::Isometry3d y_to_z = marker.origin * Eigen::AngleAxisd(EIGEN_PI / 2.0, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d pose = marker.origin;

  if (const auto* box = dynamic_cast<const g::Box*>(geometry))
  {
    m = baseMarker("box", parent, tags);
    m.extents = std::array<double, 3>{ box->getX(), box->getY(), box->getZ() };
  }
  else if (const auto* sphere = dynamic_cast<const g::Sphere*>(geometry))
  {
    m = baseMarker("sphere", parent, tags);
    m.radius = sphere->getRadius();
  }
  else if (const auto* cylinder = dynamic_cast<const g::Cylinder*>(geometry))
  {
    m = baseMarker("cylinder", parent, tags);
    m.radius = cylinder->getRadius();
    m.length = cylinder->getLength();
    pose = y_to_z;
  }
  else if (const auto* capsule = dynamic_cast<const g::Capsule*>(geometry))
  {
    m = baseMarker("capsule", parent, tags);
    m.radius = capsule->getRadius();
    m.length = capsule->getLength();
    pose = y_to_z;
  }
  else if (const auto* cone = dynamic_cast<const g::Cone*>(geometry))
  {
    m = baseMarker("cone", parent, tags);
    m.radius = cone->getRadius();
    m.length = cone->getLength();
    pose = y_to_z;
  }
  else
  {
    TESSERACT_LOG_WARN("tesseract_visualization_threejs: geometry marker type {} cannot be shown by the client",
                       geometry != nullptr ? static_cast<int>(geometry->getType()) : -1);
    return;
  }

  m.position = toArray(pose.translation());
  m.quaternion = toWxyz(pose);
  out.push_back(std::move(m));
}

void appendToolpath(std::vector<ViewerMarker>& out,
                    const vis::ToolpathMarker& marker,
                    const std::string& parent,
                    const std::vector<std::string>& tags)
{
  for (const auto& segment : marker.toolpath)
  {
    if (marker.show_path && segment.size() > 1)
    {
      ViewerMarker line = baseMarker("lines", parent, tags);
      line.color = { 0, 1, 0, 1 };
      line.linewidth = 0.001;
      std::vector<std::array<double, 3>> vertices;
      vertices.reserve(segment.size());
      for (const auto& pose : segment)
        vertices.push_back(toArray(pose.translation()));
      line.vertices = std::move(vertices);
      out.push_back(std::move(line));
    }
    if (marker.show_axis)
    {
      for (const auto& pose : segment)
      {
        ViewerMarker axes = baseMarker("axes", parent, tags);
        axes.position = toArray(pose.translation());
        axes.quaternion = toWxyz(pose);
        axes.size = marker.scale.x();
        out.push_back(std::move(axes));
      }
    }
  }
}

// Same colors and arrows as tesseract's Ignition visualization
void appendContactResults(std::vector<ViewerMarker>& out,
                          const vis::ContactResultsMarker& marker,
                          const std::string& parent,
                          const std::vector<std::string>& tags)
{
  const std::array<double, 4> sweep_color_0{ 0, 0, 1, 1 };
  const std::array<double, 4> sweep_color_1{ 0, 0, 0.5, 1 };
  namespace c = tesseract::collision;

  const auto selected = [&marker](const tesseract::common::LinkId& id) {
    return std::find(marker.link_ids.begin(), marker.link_ids.end(), id) != marker.link_ids.end();
  };

  for (const auto& result : marker.dist_results)
  {
    const double margin = marker.margin_fn ? marker.margin_fn(result.link_ids[0], result.link_ids[1]) :
                                             marker.margin_data.getCollisionMargin(result.link_ids[0], result.link_ids[1]);
    std::array<double, 4> color{ 0, 1, 0, 1 };
    if (result.distance < 0)
      color = { 1, 0, 0, 1 };
    else if (result.distance < margin)
      color = { 1, 1, 0, 1 };

    if (result.cc_type[0] == c::ContinuousCollisionType::CCType_Between)
      appendArrowBetween(out,
                         result.transform[0] * result.nearest_points_local[0],
                         result.cc_transform[0] * result.nearest_points_local[0],
                         sweep_color_0,
                         parent,
                         tags);
    if (result.cc_type[1] == c::ContinuousCollisionType::CCType_Between)
      appendArrowBetween(out,
                         result.transform[1] * result.nearest_points_local[1],
                         result.cc_transform[1] * result.nearest_points_local[1],
                         sweep_color_1,
                         parent,
                         tags);

    const bool first = selected(result.link_ids[0]);
    const bool second = selected(result.link_ids[1]);
    if (first && second)
    {
      appendArrowBetween(out, result.nearest_points[0], result.nearest_points[1], color, parent, tags);
      appendArrowBetween(out, result.nearest_points[1], result.nearest_points[0], color, parent, tags);
    }
    else if (first)
    {
      appendArrowBetween(out, result.nearest_points[1], result.nearest_points[0], color, parent, tags);
    }
    else
    {
      appendArrowBetween(out, result.nearest_points[0], result.nearest_points[1], color, parent, tags);
    }
  }
}
}  // namespace

std::string markersToJson(const std::vector<ViewerMarker>& markers)
{
  Json::Value list(Json::arrayValue);
  for (const auto& m : markers)
  {
    Json::Value v;
    v["marker_type"] = m.marker_type;
    v["parent_link_name"] = m.parent_link_name;
    v["position"] = toJsonArray(m.position);
    v["quaternion"] = toJsonArray(m.quaternion);
    v["name"] = m.name;
    v["color"] = toJsonArray(m.color);
    v["tags"] = toJsonArray(m.tags);

    if (m.size)
      v["size"] = *m.size;
    if (m.extents)
      v["size"] = toJsonArray(*m.extents);
    if (m.direction)
      v["direction"] = toJsonArray(*m.direction);
    if (m.origin)
      v["origin"] = toJsonArray(*m.origin);
    if (m.length)
      v["length"] = *m.length;
    if (m.radius)
      v["radius"] = *m.radius;
    if (m.vertices)
    {
      Json::Value vertices(Json::arrayValue);
      for (const auto& vertex : *m.vertices)
        vertices.append(toJsonArray(vertex));
      v["vertices"] = std::move(vertices);
    }
    if (m.linewidth)
      v["linewidth"] = *m.linewidth;
    list.append(std::move(v));
  }

  Json::Value root;
  root["markers"] = std::move(list);
  return detail::writeJson(root);
}

std::vector<ViewerMarker> convertMarker(const vis::Marker& marker,
                                        const std::string& default_parent_link,
                                        const std::vector<std::string>& tags)
{
  const std::string& parent = marker.getParentLink().empty() ? default_parent_link : marker.getParentLink();
  std::vector<ViewerMarker> out;

  switch (marker.getType())
  {
    case static_cast<int>(vis::MarkerType::AXIS):
    {
      const auto& axis = dynamic_cast<const vis::AxisMarker&>(marker);
      ViewerMarker m = baseMarker("axes", parent, tags);
      m.position = toArray(axis.axis.translation());
      m.quaternion = toWxyz(axis.axis);
      m.size = axis.getScale().x();
      out.push_back(std::move(m));
      break;
    }
    case static_cast<int>(vis::MarkerType::ARROW):
    {
      // ArrowMarker's two-point constructor puts the pose at the middle of the arrow and points +Z
      // along it, whatever the class comment says about the base. Its pose is treated as the center.
      const auto& arrow = dynamic_cast<const vis::ArrowMarker&>(marker);
      const double length = arrow.shaft_length + arrow.head_length;
      const Eigen::Vector3d direction = arrow.pose.linear().col(2);
      std::array<double, 4> color{ 1, 1, 1, 1 };
      if (arrow.material != nullptr)
        color = { arrow.material->color[0], arrow.material->color[1], arrow.material->color[2], arrow.material->color[3] };
      out.push_back(
          arrowMarker(arrow.pose.translation() - 0.5 * length * direction, direction, length, color, parent, tags));
      break;
    }
    case static_cast<int>(vis::MarkerType::GEOMETRY):
      appendGeometry(out, dynamic_cast<const vis::GeometryMarker&>(marker), parent, tags);
      break;
    case static_cast<int>(vis::MarkerType::TOOLPATH):
      appendToolpath(out, dynamic_cast<const vis::ToolpathMarker&>(marker), parent, tags);
      break;
    case static_cast<int>(vis::MarkerType::CONTACT_RESULTS):
      appendContactResults(out, dynamic_cast<const vis::ContactResultsMarker&>(marker), parent, tags);
      break;
    default:
      TESSERACT_LOG_WARN("tesseract_visualization_threejs: marker type {} cannot be shown by the client", marker.getType());
      break;
  }
  return out;
}

}  // namespace tesseract::visualization::threejs
