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
            
            // 创建一个ROS节点句柄
            ros::NodeHandle nh("~"); // 使用私有命名空间来获取参数
            nh.getParam("pipeline0", pipeline0);
            nh.getParam("pipeline1", pipeline1);
            nh.getParam("pipeline2", pipeline2);
            nh.getParam("pipeline3", pipeline3);
            nh.getParam("camera0_topic_name", camera0_topic_name);
            nh.getParam("camera1_topic_name", camera1_topic_name);
            nh.getParam("camera2_topic_name", camera2_topic_name);
            nh.getParam("camera3_topic_name", camera3_topic_name);
            nh.getParam("camera_merges_topic_name", camera_merges_topic_name);
            nh.getParam("node_heart_beat", node_heart_beat);
            nh.getParam("node_fps", node_fps);
            init();
            command_publisher_ = nh.advertise<xian_msg_pkg::xian_spreader_images_msg>(camera_merges_topic_name, 1);
            camera0_pub = nh.advertise<sensor_msgs::Image>(camera0_topic_name, 1);
            camera1_pub = nh.advertise<sensor_msgs::Image>(camera1_topic_name, 1);
            camera2_pub = nh.advertise<sensor_msgs::Image>(camera2_topic_name, 1);
            camera3_pub = nh.advertise<sensor_msgs::Image>(camera3_topic_name, 1);
        }

        
        ~Xian_CameraDriver()
        {
            
        }
        ros::WallTimer m_timer_Main_Func;
        ros::WallTimer m_timer_HeartBeat;
        ros::WallTimer m_timer_camera0_decode;
        ros::WallTimer m_timer_camera1_decode;
        ros::WallTimer m_timer_camera2_decode;
        ros::WallTimer m_timer_camera3_decode;

        void m_timer_camera0_decode_f(const ros::WallTimerEvent& event)
        {
            try
            {
                if(camera0_open_flag == 0)
                {
                    video_0 = cv::VideoCapture(pipeline0, cv::CAP_GSTREAMER);
                    if (!video_0.isOpened()) 
                    {
                        ROS_ERROR("Failed to open GStreamer pipeline0: %s", pipeline0.c_str());
                        return;
                    }
                    tl_image = get_image_frame(video_0);
                    tl_image_cur = tl_image;
                    camera0_open_flag = 1;
                }
                else
                {
                    ros::param::get("/xian_aqc_dynamic_parameters_server/xian_is_calibrate_flag", xian_is_calibrate_flag);
                    tl_image = get_image_frame(video_0);
                    if(xian_is_calibrate_flag == 0)
                    {
                        tl_image_cur = tl_image;
                    }

                    if(tl_image_cur.empty())
                    {
                        std::cout << "Failed to obtain one image..." << std::endl;
                    }
                    else
                    {
                        img_tl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_cur).toImageMsg();
                        spreder_imgs.tl_image = *img_tl;
                        camera0_pub.publish(img_tl); 
                    }
                }
            }
            catch (...)
            {
                // 捕获所有其他类型的异常
                ROS_WARN("An unknown error occurred during image processing or publishing.");
            }
            
        }

        void m_timer_camera1_decode_f(const ros::WallTimerEvent& event)
        {
            try
            {
                if(camera1_open_flag == 0)
                {
                    video_1 = cv::VideoCapture(pipeline1, cv::CAP_GSTREAMER);
                    if (!video_1.isOpened()) 
                    {
                        ROS_ERROR("Failed to open GStreamer pipeline1: %s", pipeline1.c_str());
                        return;
                    }
                    tr_image = get_image_frame(video_1);
                    tr_image_cur = tr_image;
                    camera1_open_flag = 1;
                }
                else
                {
                    ros::param::get("/xian_aqc_dynamic_parameters_server/xian_is_calibrate_flag", xian_is_calibrate_flag);
                    tr_image = get_image_frame(video_1);
                    if(xian_is_calibrate_flag == 0)
                    {
                        tr_image_cur = tr_image;
                    }

                    if(tr_image_cur.empty())
                    {
                        std::cout << "Failed to obtain one image..." << std::endl;
                    }
                    else
                    {
                        img_tr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_cur).toImageMsg();
                        spreder_imgs.tr_image = *img_tr;
                        camera1_pub.publish(img_tr); 
                    }
                }
            }
            catch (...)
            {
                // 捕获所有其他类型的异常
                ROS_WARN("An unknown error occurred during image processing or publishing.");
            }
            
        }

        void m_timer_camera2_decode_f(const ros::WallTimerEvent& event)
        {
            try
            {
                if(camera2_open_flag == 0)
                {
                    video_2 = cv::VideoCapture(pipeline2, cv::CAP_GSTREAMER);
                    if (!video_2.isOpened()) 
                    {
                        ROS_ERROR("Failed to open GStreamer pipeline2: %s", pipeline2.c_str());
                        return;
                    }
                    bl_image = get_image_frame(video_2);
                    bl_image_cur = bl_image;
                    camera2_open_flag = 1;
                }
                else
                {
                    ros::param::get("/xian_aqc_dynamic_parameters_server/xian_is_calibrate_flag", xian_is_calibrate_flag);
                    bl_image = get_image_frame(video_2);
                    if(xian_is_calibrate_flag == 0)
                    {
                        bl_image_cur = bl_image;
                    }

                    if(bl_image_cur.empty())
                    {
                        std::cout << "Failed to obtain one image..." << std::endl;
                    }
                    else
                    {
                        img_bl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_cur).toImageMsg();
                        spreder_imgs.bl_image = *img_bl;
                        camera2_pub.publish(img_bl); 
                    }
                }
            }
            catch (...)
            {
                // 捕获所有其他类型的异常
                ROS_WARN("An unknown error occurred during image processing or publishing.");
            }
            
        }

        void m_timer_camera3_decode_f(const ros::WallTimerEvent& event)
        {
            try
            {
                if(camera3_open_flag == 0)
                {
                    video_3 = cv::VideoCapture(pipeline3, cv::CAP_GSTREAMER);
                    if (!video_3.isOpened()) 
                    {
                        ROS_ERROR("Failed to open GStreamer pipeline3: %s", pipeline3.c_str());
                        return;
                    }
                    br_image = get_image_frame(video_3);
                    br_image_cur = br_image;
                    camera3_open_flag = 1;
                }
                else
                {
                    ros::param::get("/xian_aqc_dynamic_parameters_server/xian_is_calibrate_flag", xian_is_calibrate_flag);
                    br_image = get_image_frame(video_3);
                    if(xian_is_calibrate_flag == 0)
                    {
                        br_image_cur = br_image;
                    }

                    if(br_image_cur.empty())
                    {
                        std::cout << "Failed to obtain one image..." << std::endl;
                    }
                    else
                    {
                        img_br = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_cur).toImageMsg();
                        spreder_imgs.br_image = *img_br;
                        camera3_pub.publish(img_br); 
                    }
                }
            }
            catch (...)
            {
                // 捕获所有其他类型的异常
                ROS_WARN("An unknown error occurred during image processing or publishing.");
            }
        }

        void m_timer_Main_Func_f(const ros::WallTimerEvent& event)
        {
            this->command_callback();
        }

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get(node_heart_beat, xian_spreader_cam_rtsp_ubuntu2_heart_beat); 
            std::cout << node_heart_beat << ":" << xian_spreader_cam_rtsp_ubuntu2_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set(node_heart_beat, counter);  
        }

        cv::Mat get_image_frame(cv::VideoCapture cap)
        {
            cv::Mat frame;
            if (!cap.grab()) 
            {
                ROS_WARN("Failed to grab frame, skipping...");
                
            }

            if (!cap.retrieve(frame)) 
            {
                ROS_WARN("Failed to retrieve frame, skipping...");
                
            }

            if (frame.empty()) 
            {
                ROS_WARN("Frame is empty, skipping...");
                frame = cv::Mat::zeros(height, width, CV_8UC3);
            }
            return frame;
        }


        void init()
        {
            // pipeline0 = "rtspsrc location=rtsp://192.168.1.11:8554/camera0 latency=0 ! rtph264depay ! h264parse config-interval=-1 ! nvv4l2decoder enable-max-performance=true disable-dpb=true ! nvvidconv ! video/x-raw,format=BGRx,width=1920,height=1536 ! videoconvert ! appsink sync=false";
            // pipeline1 = "rtspsrc location=rtsp://192.168.1.11:8554/camera1 latency=0 ! rtph264depay ! h264parse config-interval=-1 ! nvv4l2decoder enable-max-performance=true disable-dpb=true ! nvvidconv ! video/x-raw,format=BGRx,width=1920,height=1536 ! videoconvert ! appsink sync=false";
            // pipeline2 = "rtspsrc location=rtsp://192.168.1.11:8554/camera2 latency=0 ! rtph264depay ! h264parse config-interval=-1 ! nvv4l2decoder enable-max-performance=true disable-dpb=true ! nvvidconv ! video/x-raw,format=BGRx,width=1920,height=1536 ! videoconvert ! appsink sync=false";
            // pipeline3 = "rtspsrc location=rtsp://192.168.1.11:8554/camera3 latency=0 ! rtph264depay ! h264parse config-interval=-1 ! nvv4l2decoder enable-max-performance=true disable-dpb=true ! nvvidconv ! video/x-raw,format=BGRx,width=1920,height=1536 ! videoconvert ! appsink sync=false";
        }
        

        void command_callback()
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            std::cout << "Time:" << timeStr << std::endl;
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            
            ros::param::get(node_fps, xian_spreader_cam_rtsp_ubuntu2_fps);
            std::cout << "FPS: " << xian_spreader_cam_rtsp_ubuntu2_fps << std::endl;


            // if(xian_is_calibrate_flag == 0)
            // {
            //     tl_image_cur = tl_image;
            //     tr_image_cur = tr_image;
            //     bl_image_cur = bl_image;
            //     br_image_cur = br_image;
            // } 
            try
            {
                if(tl_image.empty() or tr_image.empty() or bl_image.empty() or br_image.empty())
                {
                    std::cout << "Failed to obtain at least one image..." << std::endl;
                }
                else
                {

                    // img_tl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_cur).toImageMsg();
                    // img_tr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_cur).toImageMsg();
                    // img_bl = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_cur).toImageMsg();
                    // img_br = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_cur).toImageMsg();
                    // spreder_imgs.tl_image = *img_tl;
                    // spreder_imgs.tr_image = *img_tr;
                    // spreder_imgs.bl_image = *img_bl;
                    // spreder_imgs.br_image = *img_br;
                    if(xian_is_calibrate_flag == 0)
                    {
                        command_publisher_.publish(spreder_imgs);
                    }
                    
                    // camera0_pub.publish(img_tl); 
                    // camera1_pub.publish(img_tr); 
                    // camera2_pub.publish(img_bl); 
                    // camera3_pub.publish(img_br); 

                }
            }
            catch (...)
            {
                // 捕获所有其他类型的异常
                ROS_WARN("An unknown error occurred during image processing or publishing.");
            }

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            ros::param::set(node_fps, 1000.0 / timediff);

        }

    private:
        ros::Publisher command_publisher_;

        ros::Publisher camera0_pub, camera1_pub, camera2_pub, camera3_pub;

        int counter = 0;
        int xian_spreader_cam_rtsp_ubuntu2_heart_beat = 0;
        int xian_is_calibrate_flag = 0;
        double xian_spreader_cam_rtsp_ubuntu2_fps = 0;
        std::string timeStr;
        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;

        int camera0_open_flag = 0;
        int camera1_open_flag = 0;
        int camera2_open_flag = 0;
        int camera3_open_flag = 0;
        int height = 1536;
        int width = 1920;
        std::string camera0_topic_name, camera1_topic_name, camera2_topic_name, camera3_topic_name, camera_merges_topic_name;
        std::string pipeline0, pipeline1, pipeline2, pipeline3;
        std::string node_heart_beat, node_fps;
        cv::VideoCapture video_0, video_1, video_2, video_3;
        cv::Mat tl_image, tr_image, bl_image, br_image;
        cv::Mat tl_image_cur, tr_image_cur, bl_image_cur, br_image_cur;
        sensor_msgs::ImagePtr img_tl, img_tr, img_bl, img_br;
        xian_msg_pkg::xian_spreader_images_msg spreder_imgs;
};

int main(int argc, char** argv) 
{

    //initial and name node
    ros::init(argc,argv,"xian_spreader_cam_rtsp_ubuntu2_node");
    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    Xian_CameraDriver xian_spreader_cam_rtsp_ubuntu2_node;
    
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_spreader_cam_rtsp_ubuntu2_node.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1), &Xian_CameraDriver::m_timer_HeartBeat_f, &xian_spreader_cam_rtsp_ubuntu2_node);
    xian_spreader_cam_rtsp_ubuntu2_node.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.05), &Xian_CameraDriver::m_timer_Main_Func_f, &xian_spreader_cam_rtsp_ubuntu2_node);

    xian_spreader_cam_rtsp_ubuntu2_node.m_timer_camera0_decode = nh_2.createWallTimer(ros::WallDuration(0.02), &Xian_CameraDriver::m_timer_camera0_decode_f, &xian_spreader_cam_rtsp_ubuntu2_node);
    xian_spreader_cam_rtsp_ubuntu2_node.m_timer_camera1_decode = nh_2.createWallTimer(ros::WallDuration(0.02), &Xian_CameraDriver::m_timer_camera1_decode_f, &xian_spreader_cam_rtsp_ubuntu2_node);
    xian_spreader_cam_rtsp_ubuntu2_node.m_timer_camera2_decode = nh_2.createWallTimer(ros::WallDuration(0.02), &Xian_CameraDriver::m_timer_camera2_decode_f, &xian_spreader_cam_rtsp_ubuntu2_node);
    xian_spreader_cam_rtsp_ubuntu2_node.m_timer_camera3_decode = nh_2.createWallTimer(ros::WallDuration(0.02), &Xian_CameraDriver::m_timer_camera3_decode_f, &xian_spreader_cam_rtsp_ubuntu2_node);
    ros::waitForShutdown();
    return 0;
}
