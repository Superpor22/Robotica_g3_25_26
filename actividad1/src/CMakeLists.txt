# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t -std=c++23)

