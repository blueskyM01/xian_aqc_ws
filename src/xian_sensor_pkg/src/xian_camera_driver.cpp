#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "zpmc_cv_control.h"

#define PORT 2813
#define NUM_IMAGES 4

class Xian_CameraDriver {
public:
    Xian_CameraDriver() : nh_(), it_(nh_) {
        std::cout << "xian_camera_driver node has been started!" << std::endl;
        init();
        command_publisher_ = nh_.advertise<xian_msg_pkg::xian_spreader_images_msg>("xian_aqc_spreader_images", 1);
        m_timer_Main_Func = nh_.createWallTimer(ros::WallDuration(0.05), &Xian_CameraDriver::m_timer_Main_Func_f, this);
        m_timer_HeartBeat = nh_.createWallTimer(ros::WallDuration(1), &Xian_CameraDriver::m_timer_HeartBeat_f, this);
    }

    ~Xian_CameraDriver() {
        if (socket_fd != -1) {
            close(socket_fd);
        }
    }

    void m_timer_Main_Func_f(const ros::WallTimerEvent& event) {
        command_callback();
    }

    void m_timer_HeartBeat_f(const ros::WallTimerEvent& event) {
        ros::param::get("/xian_aqc_dynamic_parameters_server/xian_camera_driver_heat_beat", xian_camera_driver_heat_beat); 
        std::cout << "xian_camera_driver_heat_beat: " << xian_camera_driver_heat_beat << std::endl;
        counter = counter > 1000 ? 0 : (counter + 1);
        ros::param::set("/xian_aqc_dynamic_parameters_server/xian_camera_driver_heat_beat", counter);  
    }

    void init() {
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        serv_addr.sin_addr.s_addr = inet_addr(server_ip);
    }

    void command_callback() {
        timeStr = zpmc::zpmc_get_stystem_time();
        std::cout << "Time:" << timeStr << std::endl;
        pre_time = cur_time;
        cur_time = std::chrono::high_resolution_clock::now();
        ros::param::get("/xian_aqc_dynamic_parameters_server/xian_is_calibrate_flag", xian_is_calibrate_flag);
        ros::param::get("/xian_aqc_dynamic_parameters_server/xian_camera_sensor_fps", xian_camera_sensor_fps);
        std::cout << "FPS: " << xian_camera_sensor_fps << std::endl;

        // usleep(50 * 1000); // 50 ms

        if (xian_is_calibrate_flag == 0) {
            if (socket_fd == -1 && !connect_flag) {
                close(socket_fd);
                socket_fd = -1;
            }

            if (!connect_flag) {
                printf("socket 启动中......\n");
                socket_fd = socket(AF_INET, SOCK_STREAM, 0);
                if (socket_fd == -1) {
                    perror("socket 创建失败");
                    return;
                } else {
                    usleep(1000 * 1000); // 1 second
                    tvTimeout.tv_sec = 3;
                    tvTimeout.tv_usec = 0;
                    tvTimeout_recv.tv_sec = 1;
                    tvTimeout_recv.tv_usec = 0;
                    setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &tvTimeout, sizeof(tvTimeout));
                    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tvTimeout_recv, sizeof(tvTimeout_recv));

                    // 判断是否连接成功
                    res = connect(socket_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
                    if (res != 0) {
                        perror("bind 链接失败, 再次尝试请求");
                    } else {
                        connect_flag = true; 
                        printf("bind 链接成功!\n");
                    }
                }
            }

            if (connect_flag) {
                std::vector<int> encoded_image_sizes(NUM_IMAGES);

                // Receive the sizes of the encoded images
                if (read(socket_fd, encoded_image_sizes.data(), sizeof(int) * NUM_IMAGES) <= 0) {
                    std::cerr << "Failed to receive the sizes of the encoded images" << std::endl;
                    close(socket_fd);
                    connect_flag = false;
                    return;
                }

                std::cout << "Sizes of encoded images: ";
                for (int size : encoded_image_sizes) {
                    std::cout << size << " ";
                }
                std::cout << std::endl;

                std::vector<std::vector<uchar>> encoded_images(NUM_IMAGES);
                for (int i = 0; i < NUM_IMAGES; ++i) {
                    encoded_images[i].resize(encoded_image_sizes[i]);
                    int bytes_received = 0;
                    while (bytes_received < encoded_image_sizes[i]) {
                        int current_received = read(socket_fd, encoded_images[i].data() + bytes_received, encoded_image_sizes[i] - bytes_received);
                        if (current_received <= 0) {
                            std::cerr << "Error receiving the image" << std::endl;
                            close(socket_fd);
                            connect_flag = false;
                            return;
                        }
                        bytes_received += current_received;
                    }
                    std::cout << "Total bytes received for image " << i + 1 << ": " << bytes_received << std::endl;
                }

                tl_image = cv::imdecode(encoded_images[0], cv::IMREAD_COLOR);
                tr_image = cv::imdecode(encoded_images[1], cv::IMREAD_COLOR);
                bl_image = cv::imdecode(encoded_images[2], cv::IMREAD_COLOR);
                br_image = cv::imdecode(encoded_images[3], cv::IMREAD_COLOR);
            }
        } 

        if (tl_image.empty() || tr_image.empty() || bl_image.empty() || br_image.empty()) {
            std::cout << "finished!" << std::endl;
            connect_flag = false;
            close(socket_fd);
            socket_fd = -1;
        } else {
            cv::rotate(br_image, br_image, cv::ROTATE_180);
            cv::rotate(bl_image, bl_image, cv::ROTATE_180);
            img_tl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image).toImageMsg();
            img_tr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image).toImageMsg();
            img_bl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image).toImageMsg();
            img_br = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image).toImageMsg();
            
            
            
            spreder_imgs.tl_image = *img_tl;
            spreder_imgs.tr_image = *img_bl;
            spreder_imgs.bl_image = *img_tr;
            spreder_imgs.br_image = *img_br;
            command_publisher_.publish(spreder_imgs);
        }

        elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
        timediff = elapsedTimeP.count();
        ros::param::set("/xian_aqc_dynamic_parameters_server/xian_camera_sensor_fps", 1000.0 / timediff);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher command_publisher_;
    ros::WallTimer m_timer_Main_Func;  // Declare m_timer_Main_Func as a member variable
    ros::WallTimer m_timer_HeartBeat;  // Declare m_timer_HeartBeat as a member variable
    int counter = 0;
    int xian_camera_driver_heat_beat = 0;
    int xian_is_calibrate_flag = 0;
    double xian_camera_sensor_fps = 0;
    std::string timeStr;
    std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
    std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
    std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
    double timediff = 1.0;

    cv::Mat tl_image, tr_image, bl_image, br_image;
    sensor_msgs::ImagePtr img_tl, img_tr, img_bl, img_br;
    xian_msg_pkg::xian_spreader_images_msg spreder_imgs;

    int socket_fd = -1;
    bool connect_flag = false;
    struct timeval tvTimeout;
    struct timeval tvTimeout_recv;
    int res;
    std::string ip = "192.168.1.12";
    char const* server_ip = ip.c_str();

    struct sockaddr_in serv_addr;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "xian_camera_driver_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    Xian_CameraDriver xian_camera_driver_node;
    ros::waitForShutdown();
    return 0;
}
