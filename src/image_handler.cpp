//
//  image_handler.cpp
//  ros2server
//
//  Created by Andreas Klintberg on 12/14/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//

#include "image_handler.hpp"
#include "opencv2/opencv.hpp"


ImageHandler::ImageHandler() : requests_(0) {}

void ImageHandler::OnImage(const sensor_msgs::msg::CompressedImage &image) {
    if (requests_ == 0) {
        return;
    }
    
    try {
        std::unique_lock<std::mutex> lock(mutex_);
        auto current_image = cv_bridge::toCvCopy(image, std::string());
        cv::imencode(".jpg", current_image->image, send_buffer_,
                     std::vector<int>() /* params */);
        cvar_.notify_all();
    } catch (cv_bridge::Exception &e) {
        std::cout << "Error when converting ROS image to CV image: " << e.what();
        return;
    }
}

bool ImageHandler::handleGet(CivetServer *server, struct mg_connection *conn) {
    requests_++;
    
    mg_printf(conn,
              "HTTP/1.1 200 OK\r\n"
              "Connection: close\r\n"
              "Max-Age: 0\r\n"
              "Expires: 0\r\n"
              "Cache-Control: no-cache, no-store, must-revalidate, private\r\n"
              "Pragma: no-cache\r\n"
              "Content-Type: multipart/x-mixed-replace; "
              "boundary=--BoundaryString\r\n"
              "\r\n");
    
    std::vector<uchar> to_send;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            to_send = send_buffer_;
        }
        
        if (!to_send.empty()) {
            // Sends the image data
            mg_printf(conn,
                      "--BoundaryString\r\n"
                      "Content-type: image/jpeg\r\n"
                      "Content-Length: %zu\r\n"
                      "\r\n",
                      to_send.size());
            
            if (mg_write(conn, &to_send[0], to_send.size()) <= 0) {
                requests_--;
                return false;
            }
            mg_printf(conn, "\r\n\r\n");
        }
        
        std::unique_lock<std::mutex> lock(mutex_);
        cvar_.wait(lock);
    }
    
    requests_--;
    return true;
}
