#include <openssl/sha.h>

#include <array>

#include <tesseract/visualization/threejs/conversions.h>

namespace tesseract::visualization::threejs
{
std::string hashBytes(std::string_view data)
{
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  SHA256(reinterpret_cast<const unsigned char*>(data.data()), data.size(), digest.data());

  static constexpr char digits[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(digest.size() * 2);
  for (unsigned char byte : digest)
  {
    hex.push_back(digits[byte >> 4]);
    hex.push_back(digits[byte & 0xf]);
  }
  return hex;
}

}  // namespace tesseract::visualization::threejs
