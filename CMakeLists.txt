cmake_minimum_required(VERSION 3.20)
project(HelloWorld CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM CONFIG REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/deps/gtsam-install")

add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries(HelloWorld PRIVATE depthai::core ${OpenCV_LIBS})

add_executable(Camera Camera.cpp)
target_link_libraries(Camera PRIVATE depthai::core ${OpenCV_LIBS})

add_executable(VIO main.cpp)
target_link_libraries(VIO PRIVATE depthai::core ${OpenCV_LIBS} gtsam)
# NOTE: not adding deps/eigen here on purpose - GTSAM was built with GTSAM_USE_SYSTEM_EIGEN=OFF,
# so it bundles its own Eigen copy (3.3.7) and Backend.cpp's Eigen::Vector3d types cross the
# gtsam API boundary directly. Relying on the gtsam target to transitively bring in its own
# Eigen include path avoids having two different Eigen copies visible at once. If Backend.cpp's
# `#include <Eigen/Dense>` can't be found once GTSAM finishes building, that means the gtsam
# target isn't exposing it transitively and this needs a manual include path added instead.

# Copy every runtime DLL each exe needs next to it after build, so it can be
# run directly without messing with PATH. depthai-core.dll was built against
# OpenCV 4.13.0 specifically (opencv_world4130.dll) - CMAKE_PREFIX_PATH in setup.ps1
# points at that same 4.13.0 build so this ${OpenCV_LIBS} import matches it exactly.
# libusb-1.0.dll is a delay-load dependency of depthai-core.dll not shipped by either
# package (needed for it to actually talk to the device over USB).
file(GLOB DEPTHAI_RUNTIME_DLLS "${CMAKE_CURRENT_SOURCE_DIR}/deps/depthai-core/bin/depthai-core.dll"
                                "${CMAKE_CURRENT_SOURCE_DIR}/deps/depthai-core/bin/dynamic_calibration.dll")
file(GLOB OPENCV_RUNTIME_DLLS "${CMAKE_CURRENT_SOURCE_DIR}/deps/opencv/build/x64/vc16/bin/opencv_world*.dll"
                               "${CMAKE_CURRENT_SOURCE_DIR}/deps/opencv/build/x64/vc16/bin/opencv_videoio_ffmpeg*.dll")
foreach(target HelloWorld Camera VIO)
    foreach(dll ${DEPTHAI_RUNTIME_DLLS} ${OPENCV_RUNTIME_DLLS})
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${dll}" "$<TARGET_FILE_DIR:${target}>")
    endforeach()
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libusb-1.0.dll"
            "$<TARGET_FILE_DIR:${target}>"
    )
endforeach()

# gtsam.dll (built as a shared lib) only gets pulled in via linking, not any of the
# glob-copy loops above, and VCPKG_APPLOCAL_DEPS doesn't know about it since it lives
# outside the vcpkg tree - so copy it (and its own dependency, metis-gtsam.dll) manually,
# just for VIO since it's the only target that links gtsam.
file(GLOB GTSAM_RUNTIME_DLLS "${CMAKE_CURRENT_SOURCE_DIR}/deps/gtsam-install/bin/gtsam.dll"
                              "${CMAKE_CURRENT_SOURCE_DIR}/deps/gtsam-install/bin/metis-gtsam.dll")
foreach(dll ${GTSAM_RUNTIME_DLLS})
    add_custom_command(TARGET VIO POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${dll}" "$<TARGET_FILE_DIR:VIO>")
endforeach()

# gtsam.dll itself (dumpbin /dependents) also needs boost_filesystem and boost_timer
# (boost_serialization gets auto-copied by vcpkg's VCPKG_APPLOCAL_DEPS since it's VIO's
# own direct dependency, but these two are gtsam.dll's *transitive* deps, which
# VCPKG_APPLOCAL_DEPS doesn't walk into since gtsam.dll lives outside the vcpkg tree).
file(GLOB GTSAM_BOOST_RUNTIME_DLLS
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/vcpkg/installed/x64-windows/bin/boost_filesystem*.dll"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/vcpkg/installed/x64-windows/bin/boost_timer*.dll")
foreach(dll ${GTSAM_BOOST_RUNTIME_DLLS})
    add_custom_command(TARGET VIO POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${dll}" "$<TARGET_FILE_DIR:VIO>")
endforeach()
