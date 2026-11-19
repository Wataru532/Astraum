cmake_minimum_required(VERSION 3.20)
project(Astraum C CXX RC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# raylib (vendored, static)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(OPENGL_VERSION "3.3" CACHE STRING "" FORCE)
add_subdirectory(third_party/raylib)

set(ASTRAUM_CXX_FLAGS $<$<COMPILE_LANGUAGE:CXX>:-O2 -Wall -Wno-missing-field-initializers>)
if(SF_NOSTRIP)
  set(ASTRAUM_CXX_FLAGS ${ASTRAUM_CXX_FLAGS} -g)
endif()

# ---------------------------------------------------------------- layers (clean architecture)
# The dependency rule is enforced by the build: an inner layer is a library that cannot see the include
# directories of the outer ones (nor raylib's), so a wrong #include does not compile.
#
#   domain + shared   entities and game rules: pure C++ (no raylib, no Windows, no file access)
#   application       use cases: maps / saving, settings, built-in designs; C++ standard library only
#   infrastructure    raylib, Win32, files, terrain maps, audio: implements the ports of the inner layers
#   presentation      screens, UI toolkit, input, 3D rendering
#   main.cpp          composition root: creates the window, plugs the infrastructure into the ports, runs the loop
file(GLOB DOMAIN_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/shared/*.cpp ${CMAKE_SOURCE_DIR}/src/domain/*.cpp)
add_library(astraum_domain STATIC ${DOMAIN_SOURCES})
target_include_directories(astraum_domain PUBLIC ${CMAKE_SOURCE_DIR}/src/shared ${CMAKE_SOURCE_DIR}/src/domain)
target_compile_options(astraum_domain PRIVATE ${ASTRAUM_CXX_FLAGS})

file(GLOB APPLICATION_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/application/*.cpp)
add_library(astraum_application STATIC ${APPLICATION_SOURCES})
target_include_directories(astraum_application PUBLIC ${CMAKE_SOURCE_DIR}/src/application)
target_link_libraries(astraum_application PUBLIC astraum_domain)
target_compile_options(astraum_application PRIVATE ${ASTRAUM_CXX_FLAGS})

file(GLOB OUTER_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/infrastructure/*.cpp ${CMAKE_SOURCE_DIR}/src/presentation/*.cpp
     ${CMAKE_SOURCE_DIR}/src/tests/*.cpp ${CMAKE_SOURCE_DIR}/src/main.cpp)
add_executable(Astraum WIN32 ${OUTER_SOURCES} ${CMAKE_SOURCE_DIR}/src/app.rc)
target_include_directories(Astraum PRIVATE ${CMAKE_SOURCE_DIR}/src/infrastructure ${CMAKE_SOURCE_DIR}/src/presentation ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(Astraum PRIVATE astraum_application raylib opengl32 gdi32 winmm psapi pdh shell32 ole32)
target_compile_options(Astraum PRIVATE ${ASTRAUM_CXX_FLAGS})

# embedded resources: rebuild app.rc when an asset changes
file(GLOB SF_MAPS ${CMAKE_SOURCE_DIR}/assets/maps/* ${CMAKE_SOURCE_DIR}/assets/sky/*)
file(GLOB SF_AUDIO ${CMAKE_SOURCE_DIR}/src/audio/fx/* ${CMAKE_SOURCE_DIR}/src/audio/bgm/*)
set_source_files_properties(${CMAKE_SOURCE_DIR}/src/app.rc PROPERTIES OBJECT_DEPENDS
                            "${CMAKE_SOURCE_DIR}/assets/icon.ico;${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.txt;${SF_MAPS};${SF_AUDIO}")

if(MINGW)
  if(SF_NOSTRIP)
    # diagnostic build with symbols (cmake -B build_dbg -DSF_NOSTRIP=1)
    target_link_options(Astraum PRIVATE -static -static-libgcc -static-libstdc++ -Wl,-Map=${CMAKE_BINARY_DIR}/Astraum.map)
  else()
    target_link_options(Astraum PRIVATE -static -static-libgcc -static-libstdc++ -s -Wl,-Map=${CMAKE_BINARY_DIR}/Astraum.map)
  endif()
endif()
