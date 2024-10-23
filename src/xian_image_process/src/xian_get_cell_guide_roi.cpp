#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_keypoints.h"
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "xian_msg_pkg/xian_cell_guide_roi_msg.h"
#include "zpmc_cv_control.h"


class Xian_GetCellGuideROI
{
    public:
        Xian_GetCellGuideROI()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_publisher_cell_guide_roi = nh.advertise<xian_msg_pkg::xian_cell_guide_roi_msg>("xian_cell_guide_crop_images", 1);
            command_publisher_spreader_image = nh.advertise<xian_msg_pkg::xian_spreader_images_msg>("xian_spreader_image_align_with_cell_guide_crop", 1);
            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_keypoints>("xian_aqc_keypoints", 1, &Xian_GetCellGuideROI::command_callback_keypoints, this);
            command_subscribe_spreader_images = nh.subscribe<xian_msg_pkg::xian_spreader_images_msg>("xian_aqc_spreader_images_ubuntu3", 1, &Xian_GetCellGuideROI::command_callback_spreader_images, this);
        }

        ros::WallTimer m_timer_HeartBeat;
        ros::WallTimer m_timer_MsgPub;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_get_cell_guide_roi_heart_beat", xian_get_cell_guide_roi_heart_beat); 
            std::cout << "xian_get_cell_guide_roi_heart_beat: " << xian_get_cell_guide_roi_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_get_cell_guide_roi_heart_beat", counter);  // 自行替换
        }

        void m_timer_MsgPub_f(const ros::WallTimerEvent& event)
        {
            command_publisher_cell_guide_roi.publish(crop_images);
            command_publisher_spreader_image.publish(spreader_images);
        }

    private:

        ros::Publisher command_publisher_cell_guide_roi;
        ros::Publisher command_publisher_spreader_image;
        ros::Subscriber command_subscribe_;
        ros::Subscriber command_subscribe_spreader_images;
        xian_msg_pkg::xian_cell_guide_roi_msg crop_images;
        xian_msg_pkg::xian_spreader_images_msg spreader_images;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point cur_time2 = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time2 = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP2;
        double timediff = 1.0;
        double timediff2 = 1.0;
        int counter = 0;
        int xian_get_cell_guide_roi_heart_beat = 0;
        double xian_get_cell_guide_roi_fps = 1.0;  
        std::string timeStr;

        int tl_cell_guide_crop0_cx = 0;
        int tl_cell_guide_crop0_cy = 0;
        int tr_cell_guide_crop0_cx = 0;
        int tr_cell_guide_crop0_cy = 0;
        int bl_cell_guide_crop0_cx = 0;
        int bl_cell_guide_crop0_cy = 0;
        int br_cell_guide_crop0_cx = 0;
        int br_cell_guide_crop0_cy = 0;

        int tl_cell_guide_crop1_cx = 0;
        int tl_cell_guide_crop1_cy = 0;
        int tr_cell_guide_crop1_cx = 0;
        int tr_cell_guide_crop1_cy = 0;
        int bl_cell_guide_crop1_cx = 0;
        int bl_cell_guide_crop1_cy = 0;
        int br_cell_guide_crop1_cx = 0;
        int br_cell_guide_crop1_cy = 0;

        int tl_cell_guide_crop2_cx = 0;
        int tl_cell_guide_crop2_cy = 0;
        int tr_cell_guide_crop2_cx = 0;
        int tr_cell_guide_crop2_cy = 0;
        int bl_cell_guide_crop2_cx = 0;
        int bl_cell_guide_crop2_cy = 0;
        int br_cell_guide_crop2_cx = 0;
        int br_cell_guide_crop2_cy = 0;

        int tl_container_corner_x = 0;
        int tl_container_corner_y = 0;
        int tr_container_corner_x = 0;
        int tr_container_corner_y = 0;
        int bl_container_corner_x = 0;
        int bl_container_corner_y = 0;
        int br_container_corner_x = 0;
        int br_container_corner_y = 0;

        int crop_w = 256;
        int crop_h = 256;

 
        cv::Mat tl_image, tr_image, bl_image, br_image;
        cv::Mat tl_image_cell_guide_crop_0, tr_image_cell_guide_crop_0, bl_image_cell_guide_crop_0, br_image_cell_guide_crop_0;
        cv::Mat tl_image_cell_guide_crop_1, tr_image_cell_guide_crop_1, bl_image_cell_guide_crop_1, br_image_cell_guide_crop_1;
        cv::Mat tl_image_cell_guide_crop_2, tr_image_cell_guide_crop_2, bl_image_cell_guide_crop_2, br_image_cell_guide_crop_2;
        cv::Mat tl_image_cell_guide_crop_0_resize, tr_image_cell_guide_crop_0_resize, bl_image_cell_guide_crop_0_resize, br_image_cell_guide_crop_0_resize;
        cv::Mat tl_image_cell_guide_crop_1_resize, tr_image_cell_guide_crop_1_resize, bl_image_cell_guide_crop_1_resize, br_image_cell_guide_crop_1_resize;
        cv::Mat tl_image_cell_guide_crop_2_resize, tr_image_cell_guide_crop_2_resize, bl_image_cell_guide_crop_2_resize, br_image_cell_guide_crop_2_resize;
        sensor_msgs::ImagePtr tl_cell_guide_crop_image0, tr_cell_guide_crop_image0, bl_cell_guide_crop_image0, br_cell_guide_crop_image0;
        sensor_msgs::ImagePtr tl_cell_guide_crop_image1, tr_cell_guide_crop_image1, bl_cell_guide_crop_image1, br_cell_guide_crop_image1;
        sensor_msgs::ImagePtr tl_cell_guide_crop_image2, tr_cell_guide_crop_image2, bl_cell_guide_crop_image2, br_cell_guide_crop_image2;

        cv::cuda::GpuMat tl_image_cell_guide_crop_0_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_0_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_0_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_0_gpu;
        cv::cuda::GpuMat tl_image_cell_guide_crop_1_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_1_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_1_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_1_gpu;
        cv::cuda::GpuMat tl_image_cell_guide_crop_2_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_2_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_2_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_2_gpu;
        cv::cuda::GpuMat tl_image_cell_guide_crop_0_resize_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_0_resize_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_0_resize_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_0_resize_gpu;
        cv::cuda::GpuMat tl_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat tl_image_cell_guide_crop_2_resize_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_2_resize_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_2_resize_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_2_resize_gpu;
        

        void command_callback_keypoints(const xian_msg_pkg::xian_keypointsConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_get_cell_guide_roi_fps", xian_get_cell_guide_roi_fps);
            std::cout << "Sub keypoints FPS: " << xian_get_cell_guide_roi_fps << std::endl;

            tl_cell_guide_crop0_cx = data->tl_cell_guide_crop0_x;
            tl_cell_guide_crop0_cy = data->tl_cell_guide_crop0_y;
            tr_cell_guide_crop0_cx = data->tr_cell_guide_crop0_x;
            tr_cell_guide_crop0_cy = data->tr_cell_guide_crop0_y;
            bl_cell_guide_crop0_cx = data->bl_cell_guide_crop0_x;
            bl_cell_guide_crop0_cy = data->bl_cell_guide_crop0_y;
            br_cell_guide_crop0_cx = data->br_cell_guide_crop0_x;
            br_cell_guide_crop0_cy = data->br_cell_guide_crop0_y;

            tl_cell_guide_crop1_cx = data->tl_cell_guide_crop1_x;
            tl_cell_guide_crop1_cy = data->tl_cell_guide_crop1_y;
            tr_cell_guide_crop1_cx = data->tr_cell_guide_crop1_x;
            tr_cell_guide_crop1_cy = data->tr_cell_guide_crop1_y;
            bl_cell_guide_crop1_cx = data->bl_cell_guide_crop1_x;
            bl_cell_guide_crop1_cy = data->bl_cell_guide_crop1_y;
            br_cell_guide_crop1_cx = data->br_cell_guide_crop1_x;
            br_cell_guide_crop1_cy = data->br_cell_guide_crop1_y;

            tl_cell_guide_crop2_cx = data->tl_cell_guide_crop2_x;
            tl_cell_guide_crop2_cy = data->tl_cell_guide_crop2_y;
            tr_cell_guide_crop2_cx = data->tr_cell_guide_crop2_x;
            tr_cell_guide_crop2_cy = data->tr_cell_guide_crop2_y;
            bl_cell_guide_crop2_cx = data->bl_cell_guide_crop2_x;
            bl_cell_guide_crop2_cy = data->bl_cell_guide_crop2_y;
            br_cell_guide_crop2_cx = data->br_cell_guide_crop2_x;
            br_cell_guide_crop2_cy = data->br_cell_guide_crop2_y;

            tl_container_corner_x = data->tl_container_corner_x;
            tl_container_corner_y = data->tl_container_corner_y;
            tr_container_corner_x = data->tr_container_corner_x;
            tr_container_corner_y = data->tr_container_corner_y;
            bl_container_corner_x = data->bl_container_corner_x;
            bl_container_corner_y = data->bl_container_corner_y;
            br_container_corner_x = data->br_container_corner_x;
            br_container_corner_y = data->br_container_corner_y;

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_get_cell_guide_roi_fps", 1000.0 / timediff);
            std::cout << "Time-consuming kepoints: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cur_time).count() 
                      << "ms " << std::endl;
        } 

        void command_callback_spreader_images(const xian_msg_pkg::xian_spreader_images_msgConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time2 = cur_time2;
            cur_time2 = std::chrono::high_resolution_clock::now();

            tl_image = cv_bridge::toCvShare(data->tl_image, data, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(data->tr_image, data, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(data->bl_image, data, "bgr8")->image;
            br_image = cv_bridge::toCvShare(data->br_image, data, "bgr8")->image; 
            int src_w = tl_image.cols;
            int src_h = tl_image.rows;

            // cell guide crop0
            cv::Point* clip0_cell_guide_tl_xy = zpmc::crop_area(tl_cell_guide_crop0_cx, tl_cell_guide_crop0_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip0_cell_guide_tl_xy0 = *(clip0_cell_guide_tl_xy+0);
            cv::Point clip0_cell_guide_tl_xy1 = *(clip0_cell_guide_tl_xy+1);
            tl_image_cell_guide_crop_0 = tl_image(cv::Rect(clip0_cell_guide_tl_xy0.x, clip0_cell_guide_tl_xy0.y, crop_w, crop_h)).clone();
            
            cv::Point* clip0_cell_guide_tr_xy = zpmc::crop_area(tr_cell_guide_crop0_cx, tr_cell_guide_crop0_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip0_cell_guide_tr_xy0 = *(clip0_cell_guide_tr_xy+0);
            cv::Point clip0_cell_guide_tr_xy1 = *(clip0_cell_guide_tr_xy+1);
            tr_image_cell_guide_crop_0 = tr_image(cv::Rect(clip0_cell_guide_tr_xy0.x, clip0_cell_guide_tr_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip0_cell_guide_bl_xy = zpmc::crop_area(bl_cell_guide_crop0_cx, bl_cell_guide_crop0_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip0_cell_guide_bl_xy0 = *(clip0_cell_guide_bl_xy+0);
            cv::Point clip0_cell_guide_bl_xy1 = *(clip0_cell_guide_bl_xy+1);
            bl_image_cell_guide_crop_0 = bl_image(cv::Rect(clip0_cell_guide_bl_xy0.x, clip0_cell_guide_bl_xy0.y, crop_w, crop_h)).clone();
            
            cv::Point* clip0_cell_guide_br_xy = zpmc::crop_area(br_cell_guide_crop0_cx, br_cell_guide_crop0_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip0_cell_guide_br_xy0 = *(clip0_cell_guide_br_xy+0);
            cv::Point clip0_cell_guide_br_xy1 = *(clip0_cell_guide_br_xy+1);
            br_image_cell_guide_crop_0 = br_image(cv::Rect(clip0_cell_guide_br_xy0.x, clip0_cell_guide_br_xy0.y, crop_w, crop_h)).clone();

            // cell guide crop1
            cv::Point* clip1_cell_guide_tl_xy = zpmc::crop_area(tl_cell_guide_crop1_cx, tl_cell_guide_crop1_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_tl_xy0 = *(clip1_cell_guide_tl_xy+0);
            cv::Point clip1_cell_guide_tl_xy1 = *(clip1_cell_guide_tl_xy+1);
            tl_image_cell_guide_crop_1 = tl_image(cv::Rect(clip1_cell_guide_tl_xy0.x, clip1_cell_guide_tl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip1_cell_guide_tr_xy = zpmc::crop_area(tr_cell_guide_crop1_cx, tr_cell_guide_crop1_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_tr_xy0 = *(clip1_cell_guide_tr_xy+0);
            cv::Point clip1_cell_guide_tr_xy1 = *(clip1_cell_guide_tr_xy+1);
            tr_image_cell_guide_crop_1 = tr_image(cv::Rect(clip1_cell_guide_tr_xy0.x, clip1_cell_guide_tr_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip1_cell_guide_bl_xy = zpmc::crop_area(bl_cell_guide_crop1_cx, bl_cell_guide_crop1_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_bl_xy0 = *(clip1_cell_guide_bl_xy+0);
            cv::Point clip1_cell_guide_bl_xy1 = *(clip1_cell_guide_bl_xy+1);
            bl_image_cell_guide_crop_1 = bl_image(cv::Rect(clip1_cell_guide_bl_xy0.x, clip1_cell_guide_bl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip1_cell_guide_br_xy = zpmc::crop_area(br_cell_guide_crop1_cx, br_cell_guide_crop1_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_br_xy0 = *(clip1_cell_guide_br_xy+0);
            cv::Point clip1_cell_guide_br_xy1 = *(clip1_cell_guide_br_xy+1);
            br_image_cell_guide_crop_1 = br_image(cv::Rect(clip1_cell_guide_br_xy0.x, clip1_cell_guide_br_xy0.y, crop_w, crop_h)).clone();

            // cell guide crop2
            cv::Point* clip2_cell_guide_tl_xy = zpmc::crop_area(tl_cell_guide_crop2_cx, tl_cell_guide_crop2_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_tl_xy0 = *(clip2_cell_guide_tl_xy+0);
            cv::Point clip2_cell_guide_tl_xy1 = *(clip2_cell_guide_tl_xy+1);
            tl_image_cell_guide_crop_2 = tl_image(cv::Rect(clip2_cell_guide_tl_xy0.x, clip2_cell_guide_tl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip2_cell_guide_tr_xy = zpmc::crop_area(tr_cell_guide_crop2_cx, tr_cell_guide_crop2_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_tr_xy0 = *(clip2_cell_guide_tr_xy+0);
            cv::Point clip2_cell_guide_tr_xy1 = *(clip2_cell_guide_tr_xy+1);
            tr_image_cell_guide_crop_2 = tr_image(cv::Rect(clip2_cell_guide_tr_xy0.x, clip2_cell_guide_tr_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip2_cell_guide_bl_xy = zpmc::crop_area(bl_cell_guide_crop2_cx, bl_cell_guide_crop2_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_bl_xy0 = *(clip2_cell_guide_bl_xy+0);
            cv::Point clip2_cell_guide_bl_xy1 = *(clip2_cell_guide_bl_xy+1);
            bl_image_cell_guide_crop_2 = bl_image(cv::Rect(clip2_cell_guide_bl_xy0.x, clip2_cell_guide_bl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip2_cell_guide_br_xy = zpmc::crop_area(br_cell_guide_crop2_cx, br_cell_guide_crop2_cy, crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_br_xy0 = *(clip2_cell_guide_br_xy+0);
            cv::Point clip2_cell_guide_br_xy1 = *(clip2_cell_guide_br_xy+1);
            br_image_cell_guide_crop_2 = br_image(cv::Rect(clip2_cell_guide_br_xy0.x, clip2_cell_guide_br_xy0.y, crop_w, crop_h)).clone();

            tl_image_cell_guide_crop_0_gpu.upload(tl_image_cell_guide_crop_0);
            tr_image_cell_guide_crop_0_gpu.upload(tr_image_cell_guide_crop_0);
            bl_image_cell_guide_crop_0_gpu.upload(bl_image_cell_guide_crop_0);
            br_image_cell_guide_crop_0_gpu.upload(br_image_cell_guide_crop_0);
            tl_image_cell_guide_crop_1_gpu.upload(tl_image_cell_guide_crop_1);
            tr_image_cell_guide_crop_1_gpu.upload(tr_image_cell_guide_crop_1);
            bl_image_cell_guide_crop_1_gpu.upload(bl_image_cell_guide_crop_1);
            br_image_cell_guide_crop_1_gpu.upload(br_image_cell_guide_crop_1);
            tl_image_cell_guide_crop_2_gpu.upload(tl_image_cell_guide_crop_2);
            tr_image_cell_guide_crop_2_gpu.upload(tr_image_cell_guide_crop_2);
            bl_image_cell_guide_crop_2_gpu.upload(bl_image_cell_guide_crop_2);
            br_image_cell_guide_crop_2_gpu.upload(br_image_cell_guide_crop_2);
            cv::cuda::resize(tl_image_cell_guide_crop_0_gpu, tl_image_cell_guide_crop_0_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(tr_image_cell_guide_crop_0_gpu, tr_image_cell_guide_crop_0_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(bl_image_cell_guide_crop_0_gpu, bl_image_cell_guide_crop_0_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(br_image_cell_guide_crop_0_gpu, br_image_cell_guide_crop_0_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(tl_image_cell_guide_crop_1_gpu, tl_image_cell_guide_crop_1_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(tr_image_cell_guide_crop_1_gpu, tr_image_cell_guide_crop_1_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(bl_image_cell_guide_crop_1_gpu, bl_image_cell_guide_crop_1_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(br_image_cell_guide_crop_1_gpu, br_image_cell_guide_crop_1_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(tl_image_cell_guide_crop_2_gpu, tl_image_cell_guide_crop_2_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(tr_image_cell_guide_crop_2_gpu, tr_image_cell_guide_crop_2_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(bl_image_cell_guide_crop_2_gpu, bl_image_cell_guide_crop_2_resize_gpu, cv::Size(550, 550), 2);
            cv::cuda::resize(br_image_cell_guide_crop_2_gpu, br_image_cell_guide_crop_2_resize_gpu, cv::Size(550, 550), 2);
            tl_image_cell_guide_crop_0_resize_gpu.download(tl_image_cell_guide_crop_0_resize);
            tr_image_cell_guide_crop_0_resize_gpu.download(tr_image_cell_guide_crop_0_resize);
            bl_image_cell_guide_crop_0_resize_gpu.download(bl_image_cell_guide_crop_0_resize);
            br_image_cell_guide_crop_0_resize_gpu.download(br_image_cell_guide_crop_0_resize);
            tl_image_cell_guide_crop_1_resize_gpu.download(tl_image_cell_guide_crop_1_resize);
            tr_image_cell_guide_crop_1_resize_gpu.download(tr_image_cell_guide_crop_1_resize);
            bl_image_cell_guide_crop_1_resize_gpu.download(bl_image_cell_guide_crop_1_resize);
            br_image_cell_guide_crop_1_resize_gpu.download(br_image_cell_guide_crop_1_resize);
            tl_image_cell_guide_crop_2_resize_gpu.download(tl_image_cell_guide_crop_2_resize);
            tr_image_cell_guide_crop_2_resize_gpu.download(tr_image_cell_guide_crop_2_resize);
            bl_image_cell_guide_crop_2_resize_gpu.download(bl_image_cell_guide_crop_2_resize);
            br_image_cell_guide_crop_2_resize_gpu.download(br_image_cell_guide_crop_2_resize);

            tl_cell_guide_crop_image0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_cell_guide_crop_0_resize).toImageMsg();
            tr_cell_guide_crop_image0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_cell_guide_crop_0_resize).toImageMsg();
            bl_cell_guide_crop_image0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_cell_guide_crop_0_resize).toImageMsg();
            br_cell_guide_crop_image0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_cell_guide_crop_0_resize).toImageMsg();

            tl_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_cell_guide_crop_1_resize).toImageMsg();
            tr_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_cell_guide_crop_1_resize).toImageMsg();
            bl_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_cell_guide_crop_1_resize).toImageMsg();
            br_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_cell_guide_crop_1_resize).toImageMsg();

            tl_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_cell_guide_crop_2_resize).toImageMsg();
            tr_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_cell_guide_crop_2_resize).toImageMsg();
            bl_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_cell_guide_crop_2_resize).toImageMsg();
            br_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_cell_guide_crop_2_resize).toImageMsg();

            spreader_images.tl_image = data->tl_image;
            spreader_images.tr_image = data->tr_image;
            spreader_images.bl_image = data->bl_image;
            spreader_images.br_image = data->br_image;

            crop_images.tl_cell_guide_crop0 = *tl_cell_guide_crop_image0;
            crop_images.tr_cell_guide_crop0 = *tr_cell_guide_crop_image0;
            crop_images.bl_cell_guide_crop0 = *bl_cell_guide_crop_image0;
            crop_images.br_cell_guide_crop0 = *br_cell_guide_crop_image0;

            crop_images.tl_cell_guide_crop1 = *tl_cell_guide_crop_image1;
            crop_images.tr_cell_guide_crop1 = *tr_cell_guide_crop_image1;
            crop_images.bl_cell_guide_crop1 = *bl_cell_guide_crop_image1;
            crop_images.br_cell_guide_crop1 = *br_cell_guide_crop_image1;

            crop_images.tl_cell_guide_crop2 = *tl_cell_guide_crop_image2;
            crop_images.tr_cell_guide_crop2 = *tr_cell_guide_crop_image2;
            crop_images.bl_cell_guide_crop2 = *bl_cell_guide_crop_image2;
            crop_images.br_cell_guide_crop2 = *br_cell_guide_crop_image2;

            crop_images.tl_container_corner_cx = tl_container_corner_x;
            crop_images.tl_container_corner_cy = tl_container_corner_y;
            crop_images.tr_container_corner_cx = tr_container_corner_x;
            crop_images.tr_container_corner_cy = tr_container_corner_y;
            crop_images.bl_container_corner_cx = bl_container_corner_x;
            crop_images.bl_container_corner_cy = bl_container_corner_y;
            crop_images.br_container_corner_cx = br_container_corner_x;
            crop_images.br_container_corner_cy = br_container_corner_y;

            crop_images.tl_cell_guide_crop0_tl_x = clip0_cell_guide_tl_xy0.x;
            crop_images.tl_cell_guide_crop0_tl_y = clip0_cell_guide_tl_xy0.y;
            crop_images.tr_cell_guide_crop0_tl_x = clip0_cell_guide_tr_xy0.x;
            crop_images.tr_cell_guide_crop0_tl_y = clip0_cell_guide_tr_xy0.y;
            crop_images.bl_cell_guide_crop0_tl_x = clip0_cell_guide_bl_xy0.x;
            crop_images.bl_cell_guide_crop0_tl_y = clip0_cell_guide_bl_xy0.y;
            crop_images.br_cell_guide_crop0_tl_x = clip0_cell_guide_br_xy0.x;
            crop_images.br_cell_guide_crop0_tl_y = clip0_cell_guide_br_xy0.y;
            
            crop_images.tl_cell_guide_crop1_tl_x = clip1_cell_guide_tl_xy0.x;
            crop_images.tl_cell_guide_crop1_tl_y = clip1_cell_guide_tl_xy0.y;
            crop_images.tr_cell_guide_crop1_tl_x = clip1_cell_guide_tr_xy0.x;
            crop_images.tr_cell_guide_crop1_tl_y = clip1_cell_guide_tr_xy0.y;
            crop_images.bl_cell_guide_crop1_tl_x = clip1_cell_guide_bl_xy0.x;
            crop_images.bl_cell_guide_crop1_tl_y = clip1_cell_guide_bl_xy0.y;
            crop_images.br_cell_guide_crop1_tl_x = clip1_cell_guide_br_xy0.x;
            crop_images.br_cell_guide_crop1_tl_y = clip1_cell_guide_br_xy0.y;

            crop_images.tl_cell_guide_crop2_tl_x = clip2_cell_guide_tl_xy0.x;
            crop_images.tl_cell_guide_crop2_tl_y = clip2_cell_guide_tl_xy0.y;
            crop_images.tr_cell_guide_crop2_tl_x = clip2_cell_guide_tr_xy0.x;
            crop_images.tr_cell_guide_crop2_tl_y = clip2_cell_guide_tr_xy0.y;
            crop_images.bl_cell_guide_crop2_tl_x = clip2_cell_guide_bl_xy0.x;
            crop_images.bl_cell_guide_crop2_tl_y = clip2_cell_guide_bl_xy0.y;
            crop_images.br_cell_guide_crop2_tl_x = clip2_cell_guide_br_xy0.x;
            crop_images.br_cell_guide_crop2_tl_y = clip2_cell_guide_br_xy0.y;

            // command_publisher_cell_guide_roi.publish(crop_images);
            // command_publisher_spreader_image.publish(spreader_images);
            elapsedTimeP2 = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time2 - pre_time2);
            timediff2 = elapsedTimeP2.count();
            std::cout << "Sub Spreader and crop FPS: " << 1000.0 / timediff2 << std::endl;
            std::cout << "Time-consuming crop cell guide: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cur_time2).count() 
                      << "ms " << std::endl;
        }
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_get_cell_guide_roi_node");
    Xian_GetCellGuideROI xian_get_cell_guide_roi;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_get_cell_guide_roi.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_GetCellGuideROI::m_timer_HeartBeat_f, &xian_get_cell_guide_roi);
    xian_get_cell_guide_roi.m_timer_MsgPub = nh_2.createWallTimer(ros::WallDuration(0.05), &Xian_GetCellGuideROI::m_timer_MsgPub_f, &xian_get_cell_guide_roi);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}