project(seekTest)

include_directories(${PROJECT_NAME} /usr/local/include/)

add_executable(seekTest src/seekTest.cpp)

target_link_libraries(seekTest seekware)

