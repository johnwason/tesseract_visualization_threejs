/**
 * @file threejs_visualization_plugin.cpp
 * @brief Exports ThreejsVisualization for tesseract's VisualizationLoader
 *
 * Load it by setting
 *   TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<prefix>/lib
 *   TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threejs_plugin
 *
 * VisualizationLoader returns the exported object itself, so every get() of one symbol shares one
 * server and scene. To get independent ones, construct ThreejsVisualization directly.
 */
#include <boost_plugin_loader/macros.h>

#include <tesseract/visualization/threejs/threejs_visualization.h>
#include <tesseract/visualization/threejs/threejs_visualization_anchor.h>

namespace tesseract::visualization::threejs
{
PLUGIN_ANCHOR_IMPL(ThreejsVisualizationAnchor)
}  // namespace tesseract::visualization::threejs

// TESSERACT_ADD_VISUALIZATION_PLUGIN expands to a macro that does not exist at the pinned tesseract
// commit, so the underlying boost_plugin_loader macro is used directly.
EXPORT_CLASS_SECTIONED(tesseract::visualization::threejs::ThreejsVisualization,
                       TesseractThreejsVisualizationPlugin,
                       Plotter)

// VisualizationLoader::get() with no name asks for the Ignition symbol. Exporting it too means that
// existing callers of get(), such as the tesseract_planning examples, get the viewer unchanged.
EXPORT_CLASS_SECTIONED(tesseract::visualization::threejs::ThreejsVisualization,
                       TesseractIgnitionVisualizationPlugin,
                       Plotter)
