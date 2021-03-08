cmake_minimum_required(VERSION 2.8)
project(seek)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(seekTest src/seekTest.cpp)
target_link_libraries(seekTest seekware ${OpenCV_LIBS})
add_executable(seekCalib src/seekCalib.cpp)
target_link_libraries(seekCalib ${OpenCV_LIBS})


