cmake_minimum_required(VERSION 3.20)
project(HelloWorld CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries(HelloWorld PRIVATE depthai::core ${OpenCV_LIBS})

# Copy every runtime DLL HelloWorld.exe needs next to it after build, so it can be
# run directly without messing with PATH. depthai-core.dll was built against
# OpenCV 4.13.0 specifically (opencv_world4130.dll) - CMAKE_PREFIX_PATH in setup.ps1
# points at that same 4.13.0 build so this ${OpenCV_LIBS} import matches it exactly.
# libusb-1.0.dll is a delay-load dependency of depthai-core.dll not shipped by either
# package (needed for it to actually talk to the device over USB).
file(GLOB DEPTHAI_RUNTIME_DLLS "${CMAKE_CURRENT_SOURCE_DIR}/deps/depthai-core/bin/depthai-core.dll"
                                "${CMAKE_CURRENT_SOURCE_DIR}/deps/depthai-core/bin/dynamic_calibration.dll")
file(GLOB OPENCV_RUNTIME_DLLS "${CMAKE_CURRENT_SOURCE_DIR}/deps/opencv/build/x64/vc16/bin/opencv_world*.dll"
                               "${CMAKE_CURRENT_SOURCE_DIR}/deps/opencv/build/x64/vc16/bin/opencv_videoio_ffmpeg*.dll")
foreach(dll ${DEPTHAI_RUNTIME_DLLS} ${OPENCV_RUNTIME_DLLS})
    add_custom_command(TARGET HelloWorld POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${dll}" "$<TARGET_FILE_DIR:HelloWorld>")
endforeach()
add_custom_command(TARGET HelloWorld POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libusb-1.0.dll"
        "$<TARGET_FILE_DIR:HelloWorld>"
)
