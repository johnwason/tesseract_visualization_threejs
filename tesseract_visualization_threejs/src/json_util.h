#ifndef TESSERACT_VISUALIZATION_THREEJS_JSON_UTIL_H
#define TESSERACT_VISUALIZATION_THREEJS_JSON_UTIL_H

#include <array>
#include <string>
#include <vector>

#include <json/json.h>

namespace tesseract::visualization::threejs::detail
{
inline std::string writeJson(const Json::Value& value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  builder["emitUTF8"] = true;
  return Json::writeString(builder, value);
}

template <class Sequence>
Json::Value toJsonArray(const Sequence& values)
{
  Json::Value out(Json::arrayValue);
  for (const auto& v : values)
    out.append(Json::Value(v));
  return out;
}
}  // namespace tesseract::visualization::threejs::detail

#endif  // TESSERACT_VISUALIZATION_THREEJS_JSON_UTIL_H
