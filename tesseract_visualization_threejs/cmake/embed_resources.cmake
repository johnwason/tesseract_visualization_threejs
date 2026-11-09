# Run with: cmake -DOUTPUT=out.cpp -DINPUTS=name=path,name=path -P embed_resources.cmake
# Writes each input file as a byte array and a table to find them by name, so the library needs no
# installed data files.
set(arrays "")
set(table "")
set(index 0)
string(REPLACE "," ";" entries "${INPUTS}")
foreach(entry IN LISTS entries)
  string(REPLACE "=" ";" parts "${entry}")
  list(GET parts 0 name)
  list(GET parts 1 path)
  file(READ "${path}" hex HEX)
  string(LENGTH "${hex}" hex_length)
  math(EXPR size "${hex_length} / 2")
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," bytes "${hex}")
  string(APPEND arrays "const unsigned char file_${index}[] = { ${bytes} };\n")
  string(APPEND table "  { \"${name}\", file_${index}, ${size} },\n")
  math(EXPR index "${index} + 1")
endforeach()
file(WRITE "${OUTPUT}"
"#include \"embedded_resources.h\"\nnamespace tesseract::visualization::threejs::detail\n{\nnamespace\n{\n${arrays}}\nextern const EmbeddedFile embedded_files[] = {\n${table}};\nextern const std::size_t embedded_file_count = ${index};\n}\n")
