cmake_minimum_required(VERSION 3.16)
# Prevent vcpkg from auto-deploying its DLLs and overwriting the Webots-compatible versions.
set(VCPKG_APPLOCAL_DEPS OFF CACHE BOOL "" FORCE)
project(TrackObject LANGUAGES CXX)

# ─────────────────────────────────────────────
# 0. C++ settings
# ─────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(Python3_ROOT_DIR "C:/vcpkg/installed/x64-windows/tools/python3")
# ─────────────────────────────────────────────
# 1. Packages
# ─────────────────────────────────────────────
find_package(PCL        CONFIG REQUIRED)
find_package(Eigen3     CONFIG REQUIRED)
find_package(yaml-cpp   CONFIG REQUIRED)
find_package(OpenCV            REQUIRED)
find_package(octomap    CONFIG REQUIRED)
find_package(Python3    REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11   CONFIG REQUIRED)
find_package(unofficial-sqlite3 CONFIG REQUIRED)

# ─────────────────────────────────────────────
# 2. Webots
# ─────────────────────────────────────────────
if(NOT DEFINED ENV{WEBOTS_HOME})
    message(FATAL_ERROR "WEBOTS_HOME is not set")
endif()

file(TO_CMAKE_PATH "$ENV{WEBOTS_HOME}" WEBOTS_HOME)

set(WEBOTS_CPP_INCLUDE "${WEBOTS_HOME}/include/controller/cpp")
set(WEBOTS_C_INCLUDE   "${WEBOTS_HOME}/include/controller/c")
set(WEBOTS_LIB_DIR     "${WEBOTS_HOME}/lib/controller")

message(STATUS "WEBOTS_HOME = ${WEBOTS_HOME}")
message(STATUS "WEBOTS_CPP_INCLUDE = ${WEBOTS_CPP_INCLUDE}")
message(STATUS "WEBOTS_LIB_DIR = ${WEBOTS_LIB_DIR}")

if(NOT EXISTS "${WEBOTS_CPP_INCLUDE}/webots/Robot.hpp")
    message(FATAL_ERROR "Robot.hpp not found at ${WEBOTS_CPP_INCLUDE}/webots/Robot.hpp")
endif()

if(NOT EXISTS "${WEBOTS_LIB_DIR}/Controller.lib")
    message(FATAL_ERROR "Controller.lib not found at ${WEBOTS_LIB_DIR}/Controller.lib")
endif()

# ─────────────────────────────────────────────
# 3. Webots C++ Controller executable
# הקובץ הראשי הוא main.cpp
# ─────────────────────────────────────────────
add_executable(wheelchair_cpp_controller
    main.cpp

    # Webots bridge
    WebotsBridge/WebotsDrive.cpp
    WebotsBridge/SensorProducerWebots.cpp

    # Navigation / Map
    # Navigation / Map
    MapLive/OctomapAdapter.cpp
    MapLive/MapLocal.cpp
    MapLive/AStar.cpp
    MapLive/DLite.cpp
    MapLive/WheelchairBinding.cpp

    # Sensor
    Sensor/SensorConfig.cpp

    # Lidar
    Lidar/LidarProcessing.cpp
    Lidar/LidarCloud.cpp
    Lidar/LidarProcess.cpp
    Lidar/LidarData.cpp

    # Camera
    Camera/CameraProcessing.cpp
    Camera/CameraData.cpp

    # GPS
    Gps/GpsProcessing.cpp
    Gps/GpsData.cpp
    Gps/Gps.cpp

    # Encoder
    Encoder/EncoderData.cpp
    Encoder/EncoderProcessing.cpp

    # IMU
    Imu/ImuData.cpp
    Imu/ImuProcessing.cpp

    # Radar
    Radar/RadarProcessing.cpp
    Radar/RadarData.cpp

    # Sensor Fusion
    SensorFusion/HungarianAlgorithm.cpp
    SensorFusion/SensorFusionCamera.cpp
    SensorFusion/SensorFusionData.cpp
    SensorFusion/SensorFusionLidar.cpp
    SensorFusion/SensorFusionManager.cpp
    SensorFusion/SensorFusionRadar.cpp

    # Threads
    Thread/FactorGraphManager.cpp
    Thread/SensorConsumer.cpp
    Thread/StateThread.cpp
    Thread/ThreadGrafFector.cpp
    Thread/NavigationManager.cpp

    # Tracked Objects
    TrackedObject/IcpEstimator.cpp

    # Database
    Database/FrameDatabase.cpp
)

# ─────────────────────────────────────────────
# 4. Output settings
# חשוב: שלא יכניס את ה־exe לתיקיית Debug
# ─────────────────────────────────────────────
set_target_properties(wheelchair_cpp_controller PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO

    RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers/wheelchair_cpp_controller

    RUNTIME_OUTPUT_DIRECTORY_DEBUG
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers/wheelchair_cpp_controller

    RUNTIME_OUTPUT_DIRECTORY_RELEASE
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers/wheelchair_cpp_controller
)

# ─────────────────────────────────────────────
# 5. Include directories
# ─────────────────────────────────────────────
target_include_directories(wheelchair_cpp_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}

    ${CMAKE_CURRENT_SOURCE_DIR}/Navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/MapLive
    ${CMAKE_CURRENT_SOURCE_DIR}/Sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/Lidar
    ${CMAKE_CURRENT_SOURCE_DIR}/Camera
    ${CMAKE_CURRENT_SOURCE_DIR}/Gps
    ${CMAKE_CURRENT_SOURCE_DIR}/Encoder
    ${CMAKE_CURRENT_SOURCE_DIR}/Imu
    ${CMAKE_CURRENT_SOURCE_DIR}/Radar
    ${CMAKE_CURRENT_SOURCE_DIR}/SensorFusion
    ${CMAKE_CURRENT_SOURCE_DIR}/Thread
    ${CMAKE_CURRENT_SOURCE_DIR}/TrackedObject
    ${CMAKE_CURRENT_SOURCE_DIR}/WebotsBridge
    ${CMAKE_CURRENT_SOURCE_DIR}/Database

    # תיקיות שחסרו לפי שגיאות קודמות
    ${CMAKE_CURRENT_SOURCE_DIR}/KalmanFilter
    ${CMAKE_CURRENT_SOURCE_DIR}/Frame
    ${CMAKE_CURRENT_SOURCE_DIR}/Config

    # Webots headers
    "${WEBOTS_CPP_INCLUDE}"
    "${WEBOTS_C_INCLUDE}"

    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
)

# ─────────────────────────────────────────────
# 6. Link directories
# ─────────────────────────────────────────────
target_link_directories(wheelchair_cpp_controller PRIVATE
    "${WEBOTS_LIB_DIR}"
)

# ─────────────────────────────────────────────
# 7. Link libraries
# שימי לב: אין כאן CppController.lib כי אצלך הוא לא קיים
# ─────────────────────────────────────────────
target_link_libraries(wheelchair_cpp_controller PRIVATE
    "${WEBOTS_LIB_DIR}/Controller.lib"

    ${PCL_LIBRARIES}
    Eigen3::Eigen
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp

    octomap
    octomath

    pybind11::embed
    Python3::Python

    unofficial::sqlite3::sqlite3
)

# ─────────────────────────────────────────────
# 8. Compile definitions
# ─────────────────────────────────────────────
target_compile_definitions(wheelchair_cpp_controller PRIVATE
    ${PCL_DEFINITIONS}
    WEBOTS_CONTROLLER
    PYTHON_HOME_DIR=\"${Python3_ROOT_DIR}\"
    PROJECT_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"
)

# ─────────────────────────────────────────────
# 9. Copy Python runtime DLLs to exe output directory
# python312.dll + its dependencies (zlib, libffi, ssl, etc.)
# ─────────────────────────────────────────────
set(_VCPKG_BIN "C:/vcpkg/installed/x64-windows/bin")
foreach(_dll
    python312.dll
    z.dll
    ffi-8.dll
    libssl-3-x64.dll
    libcrypto-3-x64.dll
    sqlite3.dll
    bz2.dll
    liblzma.dll
    libexpat.dll
    pcl_common.dll
    pcl_features.dll
    pcl_filters.dll
    pcl_io.dll
    pcl_io_ply.dll
    pcl_kdtree.dll
    pcl_keypoints.dll
    pcl_ml.dll
    pcl_octree.dll
    pcl_outofcore.dll
    pcl_recognition.dll
    pcl_registration.dll
    pcl_sample_consensus.dll
    pcl_search.dll
    pcl_segmentation.dll
    pcl_stereo.dll
    pcl_surface.dll
    pcl_tracking.dll
)
    if(EXISTS "${_VCPKG_BIN}/${_dll}")
        add_custom_command(TARGET wheelchair_cpp_controller POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_VCPKG_BIN}/${_dll}"
                "$<TARGET_FILE_DIR:wheelchair_cpp_controller>/${_dll}"
            COMMENT "Copying ${_dll}"
        )
    endif()
endforeach()
