cmake_minimum_required(VERSION 3.7)

project(ros2server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()


#SET(MAKE_COMPILE_FLAGS "-I/usr/local/opt/openssl/include -I/usr/local/Cellar/boost/1.68.0/include/boost")
set(CMAKE_CXX_FLAGS ${MAKE_COMPILE_FLAGS})


if(APPLE)
    set(OPENSSL_ROOT_DIR "/usr/local/opt/openssl")
endif()

find_package(OpenSSL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include /usr/local/include /usr/local/lib/ ${cv_bridge_INCLUDE_DIRS})

function(custom_executable target)
    add_executable(${target} src/${target}.cpp include/${target}.hpp)
    ament_target_dependencies(${target}
    "rclcpp"
    "sensor_msgs"
    "rcutils"
    "std_msgs"
    "cv_bridge"
    "OpenCV"
    "openssl")
    install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(server)

ament_package()
