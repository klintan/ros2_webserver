cmake_minimum_required(VERSION 3.7)

project(ros2server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()


SET(MAKE_COMPILE_FLAGS "-I/usr/local/opt/openssl/include -I/usr/local/Cellar/boost/1.68.0/include/boost")
#add_definitions(${MAKE_COMPILE_FLAGS})
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
find_package(rmw_opensplice_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(civetweb REQUIRED)

#add_subdirectory(libs/civetweb/)
include_directories(include libs/civetweb/include /usr/local/include /usr/local/lib/)
include_directories(libs/civetweb/include)

LINK_DIRECTORIES(libs/civetweb)

function(custom_executable target)
    add_executable(${target} src/${target}.cpp src/${target}.hpp)
    ament_target_dependencies(${target}
    "rclcpp"
    "sensor_msgs"
    "rcutils"
    "std_msgs"
    "OpenCV"
    "openssl"
    "simple-websocket-server")
    install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(server)


TARGET_LINK_LIBRARIES(server civetweb)

ament_package()
