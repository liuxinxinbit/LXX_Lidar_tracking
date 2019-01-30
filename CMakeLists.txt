CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

PROJECT(lidar_tracking)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O3 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)
find_package(VTK REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
find_package(OpenCV REQUIRED)

include(${VTK_USE_FILE})


set(CMAKE_AUTOUIC ON)

ADD_EXECUTABLE(lidar_tracking
    src/main.cpp
    src/PCLViewer.cpp
    src/mainwindow.cpp
)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(lidar_tracking
    Qt5::Widgets 
    ${VTK_LIBRARIES}  
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

