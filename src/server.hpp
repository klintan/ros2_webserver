//
//  server.hpp
//  ros2server
//
//  Created by Andreas Klintberg on 11/25/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//


#ifndef server_hpp
#define server_hpp

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "CivetServer.h"
#include "std_msgs/msg/string.hpp"


class WebSocketHandler : public CivetWebSocketHandler {
    public:
    
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
    
};

class ROS2WS : public rclcpp::Node {
public:
    explicit ROS2WS();
    //virtual ~ROS2WS() = default;
    void init();
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    std::unique_ptr<CivetServer> server_;
    std::unique_ptr<WebSocketHandler> websocket_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


#endif /* server_hpp */
