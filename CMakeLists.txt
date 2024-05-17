cmake_minimum_required(VERSION 3.4.1)
project("Volume_App")

set(TARGET_NAME Volume_App)

# Set the build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Add the -g flag for debugging
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

message("configure ${TARGET_NAME}")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libsynexens3-v0.7.3.0-ubuntu18.04_x64-202303101119/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libsynexens3-v0.7.3.0-ubuntu18.04_x64-202303101119/lib/ubuntu18.04_x64)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libsynexens3-v0.7.3.0-ubuntu18.04_x64-202303101119/third-party/opencv-4.4.0/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libsynexens3-v0.7.3.0-ubuntu18.04_x64-202303101119/third-party/opencv-4.4.0/lib/x64/shared)

include_directories(
    src/ToF_Camera/
    src/Facade/
    src/PointCloud/
    src/Volume/
)

if(WIN32)
  set(OpenCV_LIBS 
    opencv_core440
    opencv_imgproc440
    opencv_imgcodecs440
    opencv_highgui440
    opencv_videoio440
    opencv_calib3d440)
set(LIBSYNEXENS3 libsynexens3)

elseif(UNIX)
set(OpenCV_LIBS 
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
    opencv_videoio
    opencv_calib3d)

set(LIBSYNEXENS3 synexens3)

endif()

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp")

add_executable(Volume_App 
    src/main.cpp 
    src/ToF_Camera/ToFCamera.cpp
    src/PointCloud/PointCloud.cpp    
    src/Facade/Facade.cpp
    src/Volume/Volume.cpp
)
target_link_libraries(Volume_App  PRIVATE ${LIBSYNEXENS3} ${OpenCV_LIBS} ${PCL_LIBRARIES} Open3D::Open3D)
set_property(TARGET Volume_App PROPERTY FOLDER Samples)