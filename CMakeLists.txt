cmake_minimum_required(VERSION 3.16)

project(hog2 VERSION 0.9 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Default install prefix to ~/.local when the user hasn't set one ───────────
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local"
        CACHE PATH "Install prefix (default: ~/.local)" FORCE)
endif()

# ── GUI backend ──────────────────────────────────────────────────────────────
option(HOG2_GUI_SFML "Build GUI backend against SFML (default: ON; use OFF for headless servers)" ON)

# ── Source directories ────────────────────────────────────────────────────────
set(HOG2_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms
    ${CMAKE_CURRENT_SOURCE_DIR}/constraints
    ${CMAKE_CURRENT_SOURCE_DIR}/datastructures
    ${CMAKE_CURRENT_SOURCE_DIR}/environments
    ${CMAKE_CURRENT_SOURCE_DIR}/envutil
    ${CMAKE_CURRENT_SOURCE_DIR}/graphalgorithms
    ${CMAKE_CURRENT_SOURCE_DIR}/heuristics
    ${CMAKE_CURRENT_SOURCE_DIR}/search
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/gui
    ${CMAKE_CURRENT_SOURCE_DIR}/grids
    ${CMAKE_CURRENT_SOURCE_DIR}/mapalgorithms
)

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

set(GUI_COMMON_SRC
    gui/Common.cpp
    gui/Trackball.cpp
)

if(HOG2_GUI_SFML)
    find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)
    find_package(OpenGL REQUIRED)

    set(GUI_BACKEND_SRC
        gui/SFML/SFML_HOG.cpp
        gui/SFML/TextBox.cpp
        gui/SFML/GLUtil.cpp
    )
    set(GUI_BACKEND_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/gui/SFML)
    set(GUI_BACKEND_LIBS SFML::Graphics SFML::Window SFML::System OpenGL::GL)
else()
    set(GUI_BACKEND_SRC
        gui/STUB/STUB_HOG.cpp
    )
    set(GUI_BACKEND_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/gui/STUB)
    set(GUI_BACKEND_LIBS "")
endif()

# ── Library target ────────────────────────────────────────────────────────────
add_library(hog2 STATIC
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
    ${GUI_COMMON_SRC}
    ${GUI_BACKEND_SRC}
)

target_include_directories(hog2
    PUBLIC
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
        $<BUILD_INTERFACE:${GUI_BACKEND_INCLUDES}>
        $<INSTALL_INTERFACE:include/hog2>
)

if(GUI_BACKEND_LIBS)
    target_link_libraries(hog2 PUBLIC ${GUI_BACKEND_LIBS})
endif()

# Platform compile definitions
if(APPLE)
    target_compile_definitions(hog2 PUBLIC OS_MAC)
endif()

# ── Install rules ─────────────────────────────────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Headers – install each source directory flat into include/hog2/
foreach(_dir
    algorithms constraints datastructures environments envutil
    graphalgorithms grids heuristics mapalgorithms search simulation utils gui)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hog2
        FILES_MATCHING PATTERN "*.h"
    )
endforeach()

if(HOG2_GUI_SFML)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui/SFML/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hog2
        FILES_MATCHING PATTERN "*.h"
    )
else()
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui/STUB/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hog2
        FILES_MATCHING PATTERN "*.h"
    )
endif()

install(TARGETS hog2
    EXPORT hog2Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

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
