cmake_minimum_required(VERSION 3.16)
project(intrinsics_capture LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# --- OptiTrack Camera SDK ----------------------------------------------------
# Override with -DCAMERA_SDK_PATH=... or the CAMERA_SDK_PATH environment var.
set(CAMERA_SDK_PATH "$ENV{CAMERA_SDK_PATH}" CACHE PATH "OptiTrack Camera SDK root")
if(NOT CAMERA_SDK_PATH)
    set(CAMERA_SDK_PATH "$ENV{HOME}/optitrack/CameraSDK")
endif()
if(NOT EXISTS "${CAMERA_SDK_PATH}/lib/cmake/CameraLibrary")
    message(FATAL_ERROR
        "OptiTrack Camera SDK not found at '${CAMERA_SDK_PATH}'.\n"
        "Pass -DCAMERA_SDK_PATH=/path/to/CameraSDK.")
endif()
find_package(CameraLibrary REQUIRED HINTS "${CAMERA_SDK_PATH}")

# --- Qt6 (Qt5 will not satisfy this) -----------------------------------------
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
set(CMAKE_AUTOMOC ON)

# --- OpenCV ------------------------------------------------------------------
# aruco lives in opencv_contrib; Ubuntu's libopencv-dev ships it. calib3d is
# used for the solvePnP tilt estimate on the live view.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco)

# --- yaml-cpp ----------------------------------------------------------------
# sudo apt install libyaml-cpp-dev
find_package(yaml-cpp REQUIRED)

add_executable(intrinsics_capture
    main.cpp
    main_window.cpp
    video_view.cpp
    analysis_widgets.cpp
    capture_list.cpp
    camera_source.cpp
    board_detector.cpp
    coverage_grid.cpp
    pose_estimator.cpp
    capture_session.cpp
    solver_runner.cpp
    config.cpp
)

target_link_libraries(intrinsics_capture PRIVATE
    Qt6::Core Qt6::Gui Qt6::Widgets
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    CameraLibrary::CameraLibrary
    pthread rt jpeg
)
target_include_directories(intrinsics_capture PRIVATE ${OpenCV_INCLUDE_DIRS})

# The SDK's headers are not warning-clean; keep our own warnings loud.
target_compile_options(intrinsics_capture PRIVATE -Wall -Wextra -Wno-unused-parameter)

# setcap is cleared by every relink, so the binary loses raw-socket access each
# time it is rebuilt. `cmake --build . --target setcap` restores it.
add_custom_target(setcap
    COMMAND sudo setcap cap_net_raw,cap_net_admin+eip $<TARGET_FILE:intrinsics_capture>
    COMMAND ${CMAKE_COMMAND} -E echo "capabilities set on $<TARGET_FILE:intrinsics_capture>"
    DEPENDS intrinsics_capture
    USES_TERMINAL
    VERBATIM
)

add_custom_command(TARGET intrinsics_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo
        "-- relink cleared file capabilities; run: sudo setcap cap_net_raw,cap_net_admin+eip $<TARGET_FILE:intrinsics_capture>"
    VERBATIM
)
