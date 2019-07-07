//
//  server.hpp
//  ros2server
//
//  Created by Andreas Klintberg on 11/25/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//


#ifndef server_hpp
#define server_hpp

#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "image_handler.hpp"
#include "civetweb/CivetServer.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

class WebSocketHandler : public CivetWebSocketHandler {
public:
    using Connection = struct mg_connection;
    std::unordered_map<Connection *, std::shared_ptr<std::mutex>> connections_;

    explicit WebSocketHandler(const std::string &name) : name_(name) {}
    
    bool handleConnection(CivetServer *server,
                          const struct mg_connection *conn) override;
    void handleReadyState(CivetServer *server,
                          struct mg_connection *conn) override;
    
    bool handleData(CivetServer *server,
                    struct mg_connection *conn,
                    int bits,
                    char *data,
                    size_t data_len) override;
    void handleClose(CivetServer *server,
                     const struct mg_connection *conn) override;
    

private:
    const std::string name_;
    mutable std::mutex mutex_;
};

class ROS2WS : public rclcpp::Node {
public:
    explicit ROS2WS();
    //virtual ~ROS2WS() = default;
    void init();
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image);

    std::unique_ptr<CivetServer> server_;
    std::unique_ptr<WebSocketHandler> websocket_;
    std::unique_ptr<ImageHandler> image_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_;

    
};


#endif /* server_hpp */
