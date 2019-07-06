//
//  image_handler.hpp
//  ros2server
//
//  Created by Andreas Klintberg on 12/14/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//

#ifndef image_handler_hpp
#define image_handler_hpp

#include <atomic>
#include <string>

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

#include "civetweb/CivetServer.h"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"


/**
 * @class ImageHandler
 *
 * @brief The ImageHandler, built on top of CivetHandler, converts the received
 * ROS image message to a image stream, wrapped by MJPEG Streaming Protocol.
 */
class ImageHandler : public CivetHandler {
public:
    // The scale used to resize images sent to frontend
    static constexpr double kImageScale = 0.2;
    
    explicit ImageHandler();
    
    bool handleGet(CivetServer *server, struct mg_connection *conn);
    void OnImage(const sensor_msgs::msg::CompressedImage &image);

private:
    typedef unsigned char uchar;    
    
    std::vector<uchar> send_buffer_;
    std::atomic<int> requests_;
    
    // mutex lock and condition variable to protect the received image
    std::mutex mutex_;
    std::condition_variable cvar_;
};

#endif /* image_handler_hpp */
