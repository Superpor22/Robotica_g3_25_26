# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "generated/CMakeFiles/chocachoca_autogen.dir/AutogenUsed.txt"
  "generated/CMakeFiles/chocachoca_autogen.dir/ParseCache.txt"
  "generated/chocachoca_autogen"
  )
endif()
