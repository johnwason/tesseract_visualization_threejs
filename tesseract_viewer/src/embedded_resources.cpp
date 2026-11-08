#include "embedded_resources.h"

#include <tesseract/viewer/conversions.h>

namespace tesseract::viewer
{
std::string_view staticFile(std::string_view name)
{
  if (name == "index.html")
    return detail::asView(detail::index_html, detail::index_html_size);
  if (name == "app.js")
    return detail::asView(detail::app_js, detail::app_js_size);
  return {};
}
}  // namespace tesseract::viewer
