cmake_minimum_required(VERSION 3.16)

project(hog2 VERSION 0.9 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Default install prefix to ~/.local when the user hasn't set one ───────────
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local"
        CACHE PATH "Install prefix (default: ~/.local)" FORCE)
endif()

# ── Detect Emscripten toolchain ───────────────────────────────────────────────
if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    set(HOG2_EMSCRIPTEN ON)
else()
    set(HOG2_EMSCRIPTEN OFF)
endif()

# ── Common source files (all variants share these) ───────────────────────────
file(GLOB ALGORITHMS_SRC     algorithms/*.cpp)
file(GLOB CONSTRAINTS_SRC    constraints/*.cpp)
file(GLOB DATASTRUCTURES_SRC datastructures/*.cpp)
file(GLOB ENVIRONMENTS_SRC   environments/*.cpp)
file(GLOB ENVUTIL_SRC        envutil/*.cpp)
file(GLOB GRAPHALGORITHMS_SRC graphalgorithms/*.cpp)
file(GLOB HEURISTICS_SRC     heuristics/*.cpp)
file(GLOB UTILS_SRC          utils/*.cpp)
file(GLOB GRIDS_SRC          grids/*.cpp)
file(GLOB MAPALGORITHMS_SRC  mapalgorithms/*.cpp)

set(HOG2_COMMON_SRC
    ${ALGORITHMS_SRC}
    ${CONSTRAINTS_SRC}
    ${DATASTRUCTURES_SRC}
    ${ENVIRONMENTS_SRC}
    ${ENVUTIL_SRC}
    ${GRAPHALGORITHMS_SRC}
    ${HEURISTICS_SRC}
    ${UTILS_SRC}
    ${GRIDS_SRC}
    ${MAPALGORITHMS_SRC}
    gui/Common.cpp
    gui/Trackball.cpp
)

set(HOG2_COMMON_INCLUDES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithms>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/constraints>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/datastructures>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/environments>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/envutil>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/graphalgorithms>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/heuristics>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/search>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/simulation>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/grids>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mapalgorithms>
    $<INSTALL_INTERFACE:include/hog2>
)

# ── Platform compile definitions ─────────────────────────────────────────────
if(APPLE)
    set(HOG2_PLATFORM_DEFS OS_MAC)
endif()

# ── Helper: apply common settings to a variant target ────────────────────────
function(hog2_configure_target target)
    target_include_directories(${target} PUBLIC ${HOG2_COMMON_INCLUDES})
    if(HOG2_PLATFORM_DEFS)
        target_compile_definitions(${target} PUBLIC ${HOG2_PLATFORM_DEFS})
    endif()
    install(TARGETS ${target}
        EXPORT hog2Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endfunction()

# ── Variant: hog2-stub (headless, no external deps, always built) ─────────────
add_library(hog2-stub STATIC
    ${HOG2_COMMON_SRC}
    gui/STUB/STUB_HOG.cpp
)
set_target_properties(hog2-stub PROPERTIES OUTPUT_NAME hog2-stub)
target_include_directories(hog2-stub PUBLIC
    ${HOG2_COMMON_INCLUDES}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gui/STUB>
)
if(HOG2_PLATFORM_DEFS)
    target_compile_definitions(hog2-stub PUBLIC ${HOG2_PLATFORM_DEFS})
endif()
install(TARGETS hog2-stub EXPORT hog2Targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

# ── Variant: hog2-sfml (SFML + OpenGL, built when SFML is found) ─────────────
if(NOT HOG2_EMSCRIPTEN)
    find_package(SFML 3 COMPONENTS Graphics Window System QUIET)
    find_package(OpenGL QUIET)
    if(SFML_FOUND AND OpenGL_FOUND)
        add_library(hog2-sfml STATIC
            ${HOG2_COMMON_SRC}
            gui/SFML/SFML_HOG.cpp
            gui/SFML/TextBox.cpp
            gui/SFML/GLUtil.cpp
        )
        set_target_properties(hog2-sfml PROPERTIES OUTPUT_NAME hog2-sfml)
        target_include_directories(hog2-sfml PUBLIC
            ${HOG2_COMMON_INCLUDES}
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gui/SFML>
        )
        if(HOG2_PLATFORM_DEFS)
            target_compile_definitions(hog2-sfml PUBLIC ${HOG2_PLATFORM_DEFS})
        endif()
        target_link_libraries(hog2-sfml PUBLIC SFML::Graphics SFML::Window SFML::System OpenGL::GL)
        install(TARGETS hog2-sfml EXPORT hog2Targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
        set(HOG2_SFML_BUILT ON)
    else()
        message(STATUS "hog2: SFML or OpenGL not found — skipping hog2-sfml variant")
        set(HOG2_SFML_BUILT OFF)
    endif()
else()
    set(HOG2_SFML_BUILT OFF)
endif()

# ── Variant: hog2-web (Emscripten/WebAssembly, built when emcmake is used) ───
if(HOG2_EMSCRIPTEN)
    add_library(hog2-web STATIC
        ${HOG2_COMMON_SRC}
        gui/web/html_hog.cpp
    )
    set_target_properties(hog2-web PROPERTIES OUTPUT_NAME hog2-web)
    target_include_directories(hog2-web PUBLIC
        ${HOG2_COMMON_INCLUDES}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gui/web>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gui/STUB/GL>
    )
    target_compile_definitions(hog2-web PUBLIC NO_OPENGL)
    target_compile_options(hog2-web PUBLIC -pthread)
    target_link_options(hog2-web PUBLIC
        -pthread
        -sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency
        -sINITIAL_MEMORY=200MB
        "-sEXPORTED_FUNCTIONS=['_InitHOG','_InitHOGArg','_DoFrame','_MouseEvent','_HitKey','_SetCanvasSize']"
        "-sEXPORTED_RUNTIME_METHODS=['cwrap','stringToNewUTF8']"
        -sASSERTIONS=1
    )
    install(TARGETS hog2-web EXPORT hog2Targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

# ── Install rules ─────────────────────────────────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Common headers
foreach(_dir
    algorithms constraints datastructures environments envutil
    graphalgorithms grids heuristics mapalgorithms search simulation utils gui)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hog2
        FILES_MATCHING PATTERN "*.h"
    )
endforeach()

# Backend-specific headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui/STUB/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hog2
    FILES_MATCHING PATTERN "*.h"
)
if(HOG2_SFML_BUILT)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui/SFML/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hog2
        FILES_MATCHING PATTERN "*.h"
    )
endif()
if(HOG2_EMSCRIPTEN)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui/web/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hog2
        FILES_MATCHING PATTERN "*.h"
    )
endif()

install(EXPORT hog2Targets
    FILE hog2Targets.cmake
    NAMESPACE hog2::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hog2
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/hog2ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hog2Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/hog2Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hog2
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/hog2Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/hog2ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hog2
)
