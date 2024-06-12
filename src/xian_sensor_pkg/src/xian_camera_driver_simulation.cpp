#include<ros/ros.h>
#include <ros/callback_queue.h>
#include "boost/thread.hpp"
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include<arpa/inet.h>

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "zpmc_cv_control.h"

class Xian_CameraDriver
{
    public:
        Xian_CameraDriver()
        {
            std::cout << "xian_camera_driver node has been started!" << std::endl;
            init();
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            command_publisher_ = nh.advertise<xian_msg_pkg::xian_spreader_images_msg>("xian_aqc_spreader_images", 1);
        }

        
        ~Xian_CameraDriver()
        {
            
        }
        ros::WallTimer m_timer_Main_Func;
        ros::WallTimer m_timer_HeartBeat;

        void m_timer_Main_Func_f(const ros::WallTimerEvent& event)
        {
            this->command_callback();
        }

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_camera_driver_heat_beat", xian_camera_driver_heat_beat); 
            std::cout << "xian_camera_driver_heat_beat: " << xian_camera_driver_heat_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_camera_driver_heat_beat", counter);  
        }


        void init()
        {
            video_0.open("/root/code/cell_guide_simulator/tl.mp4"); 
            video_1.open("/root/code/cell_guide_simulator/tr.mp4");  
            video_2.open("/root/code/cell_guide_simulator/bl.mp4"); 
            video_3.open("/root/code/cell_guide_simulator/br.mp4"); 
            video_0 >> tl_image;
            video_1 >> tr_image;
            video_2 >> bl_image;
            video_3 >> br_image;
        }
        

        void command_callback()
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            std::cout << "Time:" << timeStr << std::endl;
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_is_calibrate_flag", xian_is_calibrate_flag);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_camera_sensor_fps", xian_camera_sensor_fps);
            std::cout << "FPS: " << xian_camera_sensor_fps << std::endl;


            if(xian_is_calibrate_flag == 0)
            {
                video_0 >> tl_image;
                video_1 >> tr_image;
                video_2 >> bl_image;
                video_3 >> br_image;
            } 

            if(tl_image.empty() or tr_image.empty() or bl_image.empty() or br_image.empty())
            {
                std::cout << "finished!" << std::endl;
                video_0.release();
                video_1.release();
                video_2.release();
                video_3.release();
                exit(-1);
            }
            else
            {
                img_tl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image).toImageMsg();
                img_tr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image).toImageMsg();
                img_bl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image).toImageMsg();
                img_br = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image).toImageMsg();
                spreder_imgs.tl_image = *img_tl;
                spreder_imgs.tr_image = *img_tr;
                spreder_imgs.bl_image = *img_bl;
                spreder_imgs.br_image = *img_br;
                command_publisher_.publish(spreder_imgs);
            }


            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_camera_sensor_fps", 1000.0 / timediff);

        }

    private:
        ros::Publisher command_publisher_;
        int counter = 0;
        int xian_camera_driver_heat_beat = 0;
        int xian_is_calibrate_flag = 0;
        double xian_camera_sensor_fps = 0;
        std::string timeStr;
        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;

        cv::VideoCapture video_0, video_1, video_2, video_3;
        cv::Mat tl_image, tr_image, bl_image, br_image;
        sensor_msgs::ImagePtr img_tl, img_tr, img_bl, img_br;
        xian_msg_pkg::xian_spreader_images_msg spreder_imgs;

        
};

int main(int argc, char** argv) 
{

    //initial and name node
    ros::init(argc,argv,"xian_camera_driver_node");
    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    Xian_CameraDriver xian_camera_driver_node;
    
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_camera_driver_node.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1), &Xian_CameraDriver::m_timer_HeartBeat_f, &xian_camera_driver_node);
    xian_camera_driver_node.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.05), &Xian_CameraDriver::m_timer_Main_Func_f, &xian_camera_driver_node);
    ros::waitForShutdown();
    return 0;
}
