#include "embedded_resources.h"

#include <tesseract/visualization/threejs/conversions.h>

namespace tesseract::visualization::threejs
{
namespace detail
{
std::string_view embeddedFile(std::string_view name)
{
  for (std::size_t i = 0; i < embedded_file_count; ++i)
    if (name == embedded_files[i].name)
      return { reinterpret_cast<const char*>(embedded_files[i].data), embedded_files[i].size };
  return {};
}
}  // namespace detail

std::string_view staticFile(std::string_view name)
{
  // The primitive meshes are compiled in beside the client, but are not part of it
  return name == "geometries.json" ? std::string_view{} : detail::embeddedFile(name);
}
}  // namespace tesseract::visualization::threejs
