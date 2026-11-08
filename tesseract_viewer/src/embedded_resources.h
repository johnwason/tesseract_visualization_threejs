#ifndef TESSERACT_VIEWER_EMBEDDED_RESOURCES_H
#define TESSERACT_VIEWER_EMBEDDED_RESOURCES_H

#include <cstddef>
#include <string_view>

namespace tesseract::viewer::detail
{
extern const unsigned char index_html[];
extern const std::size_t index_html_size;
extern const unsigned char app_js[];
extern const std::size_t app_js_size;
extern const unsigned char geometries_json[];
extern const std::size_t geometries_json_size;

inline std::string_view asView(const unsigned char* data, std::size_t size)
{
  return { reinterpret_cast<const char*>(data), size };
}
}  // namespace tesseract::viewer::detail

#endif  // TESSERACT_VIEWER_EMBEDDED_RESOURCES_H
