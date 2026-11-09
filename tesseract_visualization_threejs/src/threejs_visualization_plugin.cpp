/**
 * @file threejs_visualization_plugin.cpp
 * @brief Exports a factory of ThreejsVisualization for tesseract's VisualizationLoader
 *
 * Load it by setting
 *   TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<prefix>/lib
 *   TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threejs_plugin
 *
 * Each VisualizationLoader owns one instance, so one server and scene. Use another loader for an
 * independent one.
 */
#include <memory>
#include <string>

#include <tesseract/common/property_tree.h>
#include <tesseract/visualization/threejs/threejs_visualization.h>
#include <tesseract/visualization/threejs/threejs_visualization_anchor.h>
#include <tesseract/visualization/visualization_factory.h>

namespace tesseract::visualization::threejs
{
/**
 * @brief Optional configuration: host and port. Unset values come from the environment
 * @details See ThreejsVisualizationOptions for the defaults.
 */
class ThreejsVisualizationFactory : public tesseract::visualization::VisualizationFactory
{
public:
  tesseract::common::PropertyTree schema() const override
  {
    using namespace tesseract::common;
    // clang-format off
    return PropertyTreeBuilder()
        .attribute(property_attribute::TYPE, property_type::CONTAINER)
        .string("host").done()
        .int32("port").minimum(0).maximum(65535).done()
        .build();
    // clang-format on
  }

protected:
  std::unique_ptr<tesseract::visualization::Visualization>
  createImpl(const std::string& /*name*/, const tesseract::common::PropertyTree& config) const override
  {
    ThreejsVisualizationOptions options = ThreejsVisualizationOptions::fromEnvironment();
    if (const auto* host = config.find("host"); host != nullptr && !host->isNull())
      options.host = host->as<std::string>();
    if (const auto* port = config.find("port"); port != nullptr && !port->isNull())
      options.port = static_cast<unsigned short>(port->as<int>());
    return std::make_unique<ThreejsVisualization>(std::move(options));
  }
};

PLUGIN_ANCHOR_IMPL(ThreejsVisualizationAnchor)
}  // namespace tesseract::visualization::threejs

TESSERACT_ADD_VISUALIZATION_PLUGIN(tesseract::visualization::threejs::ThreejsVisualizationFactory,
                                   TesseractThreejsVisualizationPlugin)

// VisualizationLoader::get() with no name asks for the Ignition symbol. Exporting it too means that
// existing callers of get(), such as the tesseract_planning examples, get the viewer unchanged.
// If the real Gazebo plugin is also in the plugin list, library search order decides which loads.
TESSERACT_ADD_VISUALIZATION_PLUGIN(tesseract::visualization::threejs::ThreejsVisualizationFactory,
                                   TesseractIgnitionVisualizationPlugin)
