#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int PORT = 45445;
const std::string MAGIC_STRING = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
std::atomic<bool> running(true);

// 全局变量保存最新帧
cv::Mat latest_frame;
std::mutex frame_mutex;

std::string base64_encode(const std::string &input) {
    BIO *bio, *b64;
    BUF_MEM *bufferPtr;

    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new(BIO_s_mem());
    bio = BIO_push(b64, bio);

    BIO_write(bio, input.c_str(), input.length());
    BIO_flush(bio);
    BIO_get_mem_ptr(bio, &bufferPtr);

    std::string result(bufferPtr->data, bufferPtr->length);
    BIO_free_all(bio);

    if (!result.empty() && result.back() == '\n') {
        result.pop_back();
    }
    return result;
}

std::string generate_accept_key(const std::string &client_key) {
    std::string combined = client_key + MAGIC_STRING;
    unsigned char hash[SHA_DIGEST_LENGTH];
    SHA1(reinterpret_cast<const unsigned char*>(combined.c_str()), combined.size(), hash);
    return base64_encode(std::string(reinterpret_cast<char*>(hash), SHA_DIGEST_LENGTH));
}

void handle_websocket_handshake(int client_socket, const std::string &request) {
    std::string client_key;
    size_t key_pos = request.find("Sec-WebSocket-Key: ");
    if (key_pos != std::string::npos) {
        size_t end_pos = request.find("\r\n", key_pos);
        client_key = request.substr(key_pos + 19, end_pos - (key_pos + 19));
    }
    std::string accept_key = generate_accept_key(client_key);
    std::string response =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + accept_key + "\r\n\r\n";
    send(client_socket, response.c_str(), response.size(), 0);
}

std::string create_websocket_frame(const std::vector<uchar> &data, bool is_binary = true) {
    std::string frame;
    frame.push_back(static_cast<char>(0x80 | (is_binary ? 0x02 : 0x01)));
    if (data.size() <= 125) {
        frame.push_back(static_cast<char>(data.size()));
    } else if (data.size() <= 65535) {
        frame.push_back(126);
        frame.push_back(static_cast<char>((data.size() >> 8) & 0xFF));
        frame.push_back(static_cast<char>(data.size() & 0xFF));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; --i) {
            frame.push_back(static_cast<char>((data.size() >> (8 * i)) & 0xFF));
        }
    }
    frame.insert(frame.end(), data.begin(), data.end());
    return frame;
}

// ROS 图像回调，保存最新帧
void image_cb(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::lock_guard<std::mutex> lock(frame_mutex);
        latest_frame = cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void send_video_stream(int client_socket) {
    while (running) {
        cv::Mat frame_copy;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (latest_frame.empty()) {
                // 没有帧，等待
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            frame_copy = latest_frame.clone();
        }
        // 编码为JPEG
        std::vector<uchar> buffer;
        cv::imencode(".jpg", frame_copy, buffer, {cv::IMWRITE_JPEG_QUALITY, 70});
        std::string frame_data = create_websocket_frame(buffer);
        ssize_t sent = send(client_socket, frame_data.c_str(), frame_data.size(), 0);
        if (sent <= 0) break; // 客户端断开
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30fps
    }
}

void handle_client(int client_socket) {
    char buffer[4096];
    int bytes_received = recv(client_socket, buffer, sizeof(buffer), 0);
    if (bytes_received > 0) {
        std::string request(buffer, bytes_received);
        if (request.find("Upgrade: websocket") != std::string::npos) {
            handle_websocket_handshake(client_socket, request);
            send_video_stream(client_socket);
        }
    }
    close(client_socket);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "websocket_stream");
    ros::NodeHandle nh("~");
    nh.param("port", PORT, 45445);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, image_cb);

    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);
    if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(server_socket);
        return 1;
    }
    if (listen(server_socket, 5) < 0) {
        std::cerr << "Failed to listen on socket" << std::endl;
        close(server_socket);
        return 1;
    }
    std::cout << "WebSocket server listening on port " << PORT << std::endl;

    // handle ROS spin in another thread
    std::thread ros_thread([]() { ros::spin(); });

    while (running) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
        if (client_socket < 0) {
            std::cerr << "Failed to accept client connection" << std::endl;
            continue;
        }
        std::cout << "Client connected: " << inet_ntoa(client_addr.sin_addr) << std::endl;
        std::thread client_thread(handle_client, client_socket);
        client_thread.detach();
    }
    close(server_socket);
    ros_thread.join();
    return 0;
}