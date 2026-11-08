/**
 * @file threejs_visualization_anchor.h
 * @brief Anchor for packaged builds. Calling it links the plugin library into the caller, which is
 * what makes wheel repair tools (auditwheel, delvewheel, delocate) bundle the plugin.
 */
#ifndef TESSERACT_VISUALIZATION_THREEJS_VIEWER_VISUALIZATION_ANCHOR_H
#define TESSERACT_VISUALIZATION_THREEJS_VIEWER_VISUALIZATION_ANCHOR_H

#include <boost_plugin_loader/macros.h>

namespace tesseract::visualization::threejs
{
PLUGIN_ANCHOR_DECL(ThreejsVisualizationAnchor)
}

#endif  // TESSERACT_VISUALIZATION_THREEJS_VIEWER_VISUALIZATION_ANCHOR_H
