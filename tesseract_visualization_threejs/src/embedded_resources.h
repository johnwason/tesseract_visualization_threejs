#ifndef TESSERACT_VISUALIZATION_THREEJS_EMBEDDED_RESOURCES_H
#define TESSERACT_VISUALIZATION_THREEJS_EMBEDDED_RESOURCES_H

#include <cstddef>
#include <string_view>

namespace tesseract::visualization::threejs::detail
{
struct EmbeddedFile
{
  const char* name;
  const unsigned char* data;
  std::size_t size;
};

extern const EmbeddedFile embedded_files[];
extern const std::size_t embedded_file_count;

/** @brief Contents of a compiled-in file by name, or empty if there is none */
std::string_view embeddedFile(std::string_view name);
}  // namespace tesseract::visualization::threejs::detail

#endif  // TESSERACT_VISUALIZATION_THREEJS_EMBEDDED_RESOURCES_H
