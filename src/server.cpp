//
//  server.cpp
//  ros2server
//
//  Created by Andreas Klintberg on 11/25/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//

#include "server.hpp"
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#define DOCUMENT_ROOT "."
#define PORT "8081"
#define EXAMPLE_URI "/example"
#define EXIT_URI "/exit"


using std::placeholders::_1;

bool exitNow = false;


bool WebSocketHandler::handleConnection(CivetServer *server,
                                        const struct mg_connection *conn) {
    printf("WS connected\n");
    
    
    // start reading and sending ROS messages
    
    return true;
}

void WebSocketHandler::handleReadyState(CivetServer *server,
                                        struct mg_connection *conn) {
    printf("WS ready\n");
    
    const char *text = "Hello from the websocket ready handler";
    mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, text, strlen(text));
}

bool WebSocketHandler::handleData(CivetServer *server,
                                  struct mg_connection *conn,
                                  int bits,
                                  char *data,
                                  size_t data_len) {
    printf("WS got %lu bytes: ", (long unsigned)data_len);
    fwrite(data, 1, data_len, stdout);
    printf("\n");
    
    mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, data, data_len);
    return (data_len<4);
}

void WebSocketHandler::handleClose(CivetServer *server,
                                   const struct mg_connection *conn) {
    printf("WS closed\n");
}


ROS2WS::ROS2WS() : Node("Ros2Server") {
    init();
    subscription_ = this->create_subscription<std_msgs::msg::String>("/debugger", std::bind(&ROS2WS::topic_callback, this, _1));
}

void ROS2WS::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    //websocket_.get()->handleData(websocket_.get()->Connection *conn, msg->data.size(), msg->data.c_str());
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
}

void ROS2WS::init() {
    printf("Init\n");
    
    std::vector<std::string> options = {"listening_ports", "8081"};
    
    server_.reset(new CivetServer(options));
    websocket_.reset(new WebSocketHandler("/debugger"));
    
    server_->addWebSocketHandler("/debugger", *websocket_);
}


int main(int argc, char *argv[])
{
    printf("Run ROS2Server\n");
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ROS2WS>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}
