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
    
    std::unique_lock<std::mutex> lock(mutex_);
    connections_.emplace(conn, std::make_shared<std::mutex>());
    
    std::cout<< name_
    << ": Accepted connection. Total connections: " << connections_.size();
    
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
    
    std::string debugger_topic = "data";
    std::string image_topic = "image/compressed";

    subscription_ = this->create_subscription<std_msgs::msg::String>(debugger_topic, std::bind(&ROS2WS::topic_callback, this, _1));
    
    image_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(image_topic, std::bind(&ROS2WS::image_callback, this, _1));
}


void ROS2WS::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    std::cout << "Image recieved" << std::endl;
    image_.get()->OnImage(*msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr image_subscription_;
    
}

void ROS2WS::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    for (auto &kv : websocket_.get()->connections_) {
        WebSocketHandler::Connection *conn = kv.first;
        
        json json_message;
        
        json_message["topic"] = "/data";
        json_message["data"] = msg->data.c_str();
        
        std::string json_string = json_message.dump();
        
        char *data_char = const_cast<char *>(json_string.c_str());
        websocket_.get()->handleData(server_.get(), conn, 0x81, data_char, json_string.size());
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
}

void ROS2WS::init() {
    printf("Init\n");
    
    std::vector<std::string> options = {"listening_ports", "8081"};
    
    server_.reset(new CivetServer(options));
    websocket_.reset(new WebSocketHandler("/ws"));
    image_.reset(new ImageHandler());

    server_->addWebSocketHandler("/ws", *websocket_);
    server_->addHandler("/image", *image_);

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
