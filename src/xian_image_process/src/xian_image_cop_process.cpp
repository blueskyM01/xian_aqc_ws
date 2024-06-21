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
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "xian_msg_pkg/xian_crop_image_msg.h"
#include "zpmc_cv_control.h"


class Xian_ContainerCornerCopProcess
{
    public:
        Xian_ContainerCornerCopProcess()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_publisher_ = nh.advertise<xian_msg_pkg::xian_crop_image_msg>("xian_crop_images", 1);
            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_spreader_images_msg>("xian_aqc_spreader_images", 1, &Xian_ContainerCornerCopProcess::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_container_corner_cop_process_heart_beat", xian_container_corner_cop_process_heart_beat); 
            std::cout << "xian_container_corner_cop_process_heart_beat: " << xian_container_corner_cop_process_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_container_corner_cop_process_heart_beat", counter);  // 自行替换
        }

    private:

        ros::Publisher command_publisher_;
        ros::Subscriber command_subscribe_;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        int xian_container_corner_cop_process_heart_beat = 0;
        double xian_container_corner_cop_process_fps = 1.0;  
        std::string timeStr;

        int xian_tl_container_point_x = 0;
        int xian_tl_container_point_y = 0;
        int xian_tr_container_point_x = 0;
        int xian_tr_container_point_y = 0;
        int xian_bl_container_point_x = 0;
        int xian_bl_container_point_y = 0;
        int xian_br_container_point_x = 0;
        int xian_br_container_point_y = 0;
        int xian_crop_bais_x_tl_1 = 0;
        int xian_crop_bais_y_tl_1 = 0;
        int xian_crop_bais_x_tr_1 = 0;
        int xian_crop_bais_y_tr_1 = 0;
        int xian_crop_bais_x_bl_1 = 0;
        int xian_crop_bais_y_bl_1 = 0;
        int xian_crop_bais_x_br_1 = 0;
        int xian_crop_bais_y_br_1 = 0;
        int xian_crop_bais_x_tl_2 = 0;
        int xian_crop_bais_y_tl_2 = 0;
        int xian_crop_bais_x_tr_2 = 0;
        int xian_crop_bais_y_tr_2 = 0;
        int xian_crop_bais_x_bl_2 = 0;
        int xian_crop_bais_y_bl_2 = 0;
        int xian_crop_bais_x_br_2 = 0;
        int xian_crop_bais_y_br_2 = 0;

        int crop_w = 256;
        int crop_h = 256;

 
        cv::Mat tl_image, tr_image, bl_image, br_image;
        cv::Mat tl_image_crop, tr_image_crop, bl_image_crop, br_image_crop;
        cv::Mat tl_image_cell_guide_crop_1, tr_image_cell_guide_crop_1, bl_image_cell_guide_crop_1, br_image_cell_guide_crop_1;
        cv::Mat tl_image_cell_guide_crop_2, tr_image_cell_guide_crop_2, bl_image_cell_guide_crop_2, br_image_cell_guide_crop_2;
        cv::Mat tl_image_crop_resize, tr_image_crop_resize, bl_image_crop_resize, br_image_crop_resize;
        cv::Mat tl_image_cell_guide_crop_1_resize, tr_image_cell_guide_crop_1_resize, bl_image_cell_guide_crop_1_resize, br_image_cell_guide_crop_1_resize;
        cv::Mat tl_image_cell_guide_crop_2_resize, tr_image_cell_guide_crop_2_resize, bl_image_cell_guide_crop_2_resize, br_image_cell_guide_crop_2_resize;
        sensor_msgs::ImagePtr tl_crop_image, tr_crop_image, bl_crop_image, br_crop_image;
        sensor_msgs::ImagePtr tl_cell_guide_crop_image1, tr_cell_guide_crop_image1, bl_cell_guide_crop_image1, br_cell_guide_crop_image1;
        sensor_msgs::ImagePtr tl_cell_guide_crop_image2, tr_cell_guide_crop_image2, bl_cell_guide_crop_image2, br_cell_guide_crop_image2;
        cv::cuda::GpuMat tl_image_crop_gpu, tl_image_crop_resize_gpu;
        cv::cuda::GpuMat tr_image_crop_gpu, tr_image_crop_resize_gpu;
        cv::cuda::GpuMat bl_image_crop_gpu, bl_image_crop_resize_gpu;
        cv::cuda::GpuMat br_image_crop_gpu, br_image_crop_resize_gpu;
        cv::cuda::GpuMat tl_image_cell_guide_crop_1_gpu, tl_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_1_gpu, tr_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_1_gpu, bl_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_1_gpu, br_image_cell_guide_crop_1_resize_gpu;
        cv::cuda::GpuMat tl_image_cell_guide_crop_2_gpu, tl_image_cell_guide_crop_2_resize_gpu;
        cv::cuda::GpuMat tr_image_cell_guide_crop_2_gpu, tr_image_cell_guide_crop_2_resize_gpu;
        cv::cuda::GpuMat bl_image_cell_guide_crop_2_gpu, bl_image_cell_guide_crop_2_resize_gpu;
        cv::cuda::GpuMat br_image_cell_guide_crop_2_gpu, br_image_cell_guide_crop_2_resize_gpu;
        xian_msg_pkg::xian_crop_image_msg crop_images;

        void command_callback(const xian_msg_pkg::xian_spreader_images_msgConstPtr& xian_spreader_images)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_container_corner_cop_process_fps", xian_container_corner_cop_process_fps);
            std::cout << "FPS: " << xian_container_corner_cop_process_fps << std::endl;

            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_x", xian_tl_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_y", xian_tl_container_point_y);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_x", xian_tr_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_y", xian_tr_container_point_y);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_x", xian_bl_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_y", xian_bl_container_point_y);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_br_container_point_x", xian_br_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_br_container_point_y", xian_br_container_point_y);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tl_1", xian_crop_bais_x_tl_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tl_1", xian_crop_bais_y_tl_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tr_1", xian_crop_bais_x_tr_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tr_1", xian_crop_bais_y_tr_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_bl_1", xian_crop_bais_x_bl_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_bl_1", xian_crop_bais_y_bl_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_br_1", xian_crop_bais_x_br_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_br_1", xian_crop_bais_y_br_1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tl_2", xian_crop_bais_x_tl_2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tl_2", xian_crop_bais_y_tl_2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tr_2", xian_crop_bais_x_tr_2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tr_2", xian_crop_bais_y_tr_2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_bl_2", xian_crop_bais_x_bl_2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_bl_2", xian_crop_bais_y_bl_2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_br_2", xian_crop_bais_x_br_2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_br_2", xian_crop_bais_y_br_2);

            tl_image = cv_bridge::toCvShare(xian_spreader_images->tl_image, xian_spreader_images, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(xian_spreader_images->tr_image, xian_spreader_images, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(xian_spreader_images->bl_image, xian_spreader_images, "bgr8")->image;
            br_image = cv_bridge::toCvShare(xian_spreader_images->br_image, xian_spreader_images, "bgr8")->image; 
            int src_w = tl_image.cols;
            int src_h = tl_image.rows;

            // container corner crop
            cv::Point* tl_xy = zpmc::crop_area(xian_tl_container_point_x, xian_tl_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::Point tl_xy0 = *(tl_xy+0);
            cv::Point tl_xy1 = *(tl_xy+1);
            tl_image_crop = tl_image(cv::Rect(tl_xy0.x, tl_xy0.y, crop_w, crop_h)).clone();
            
            cv::Point* tr_xy = zpmc::crop_area(xian_tr_container_point_x, xian_tr_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::Point tr_xy0 = *(tr_xy+0);
            cv::Point tr_xy1 = *(tr_xy+1);
            tr_image_crop = tr_image(cv::Rect(tr_xy0.x, tr_xy0.y, crop_w, crop_h)).clone();

            cv::Point* bl_xy = zpmc::crop_area(xian_bl_container_point_x, xian_bl_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::Point bl_xy0 = *(bl_xy+0);
            cv::Point bl_xy1 = *(bl_xy+1);
            bl_image_crop = bl_image(cv::Rect(bl_xy0.x, bl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* br_xy = zpmc::crop_area(xian_br_container_point_x, xian_br_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::Point br_xy0 = *(br_xy+0);
            cv::Point br_xy1 = *(br_xy+1);
            br_image_crop = br_image(cv::Rect(br_xy0.x, br_xy0.y, crop_w, crop_h)).clone();

            // cell guide crop1
            cv::Point* clip1_cell_guide_tl_xy = zpmc::crop_area(xian_tl_container_point_x - xian_crop_bais_x_tl_1, 
                                                                xian_tl_container_point_y + xian_crop_bais_y_tl_1, 
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_tl_xy0 = *(clip1_cell_guide_tl_xy+0);
            cv::Point clip1_cell_guide_tl_xy1 = *(clip1_cell_guide_tl_xy+1);
            tl_image_cell_guide_crop_1 = tl_image(cv::Rect(clip1_cell_guide_tl_xy0.x, clip1_cell_guide_tl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip1_cell_guide_tr_xy = zpmc::crop_area(xian_tr_container_point_x + xian_crop_bais_x_tr_1, 
                                                                xian_tr_container_point_y + xian_crop_bais_y_tr_1, 
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_tr_xy0 = *(clip1_cell_guide_tr_xy+0);
            cv::Point clip1_cell_guide_tr_xy1 = *(clip1_cell_guide_tr_xy+1);
            tr_image_cell_guide_crop_1 = tr_image(cv::Rect(clip1_cell_guide_tr_xy0.x, clip1_cell_guide_tr_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip1_cell_guide_bl_xy = zpmc::crop_area(xian_bl_container_point_x - xian_crop_bais_x_bl_1,
                                                                xian_bl_container_point_y - xian_crop_bais_y_bl_1,
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_bl_xy0 = *(clip1_cell_guide_bl_xy+0);
            cv::Point clip1_cell_guide_bl_xy1 = *(clip1_cell_guide_bl_xy+1);
            bl_image_cell_guide_crop_1 = bl_image(cv::Rect(clip1_cell_guide_bl_xy0.x, clip1_cell_guide_bl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip1_cell_guide_br_xy = zpmc::crop_area(xian_br_container_point_x + xian_crop_bais_x_br_1,
                                                                xian_br_container_point_y - xian_crop_bais_y_br_1,
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip1_cell_guide_br_xy0 = *(clip1_cell_guide_br_xy+0);
            cv::Point clip1_cell_guide_br_xy1 = *(clip1_cell_guide_br_xy+1);
            br_image_cell_guide_crop_1 = br_image(cv::Rect(clip1_cell_guide_br_xy0.x, clip1_cell_guide_br_xy0.y, crop_w, crop_h)).clone();

            // cell guide crop2
            cv::Point* clip2_cell_guide_tl_xy = zpmc::crop_area(xian_tl_container_point_x - xian_crop_bais_x_tl_2, 
                                                                xian_tl_container_point_y + xian_crop_bais_y_tl_2, 
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_tl_xy0 = *(clip2_cell_guide_tl_xy+0);
            cv::Point clip2_cell_guide_tl_xy1 = *(clip2_cell_guide_tl_xy+1);
            tl_image_cell_guide_crop_2 = tl_image(cv::Rect(clip2_cell_guide_tl_xy0.x, clip2_cell_guide_tl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip2_cell_guide_tr_xy = zpmc::crop_area(xian_tr_container_point_x + xian_crop_bais_x_tr_2, 
                                                                xian_tr_container_point_y + xian_crop_bais_y_tr_2, 
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_tr_xy0 = *(clip2_cell_guide_tr_xy+0);
            cv::Point clip2_cell_guide_tr_xy1 = *(clip2_cell_guide_tr_xy+1);
            tr_image_cell_guide_crop_2 = tr_image(cv::Rect(clip2_cell_guide_tr_xy0.x, clip2_cell_guide_tr_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip2_cell_guide_bl_xy = zpmc::crop_area(xian_bl_container_point_x - xian_crop_bais_x_bl_2,
                                                                xian_bl_container_point_y - xian_crop_bais_y_bl_2,
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_bl_xy0 = *(clip2_cell_guide_bl_xy+0);
            cv::Point clip2_cell_guide_bl_xy1 = *(clip2_cell_guide_bl_xy+1);
            bl_image_cell_guide_crop_2 = bl_image(cv::Rect(clip2_cell_guide_bl_xy0.x, clip2_cell_guide_bl_xy0.y, crop_w, crop_h)).clone();

            cv::Point* clip2_cell_guide_br_xy = zpmc::crop_area(xian_br_container_point_x + xian_crop_bais_x_br_2,
                                                                xian_br_container_point_y - xian_crop_bais_y_br_2,
                                                                crop_w, crop_h, src_w, src_h);
            cv::Point clip2_cell_guide_br_xy0 = *(clip2_cell_guide_br_xy+0);
            cv::Point clip2_cell_guide_br_xy1 = *(clip2_cell_guide_br_xy+1);
            br_image_cell_guide_crop_2 = br_image(cv::Rect(clip2_cell_guide_br_xy0.x, clip2_cell_guide_br_xy0.y, crop_w, crop_h)).clone();

            // cv::resize(tl_image_crop, tl_image_crop_resize, cv::Size(512, 512), 2);
            // cv::resize(tr_image_crop, tr_image_crop_resize, cv::Size(512, 512), 2);
            // cv::resize(bl_image_crop, bl_image_crop_resize, cv::Size(512, 512), 2);
            // cv::resize(br_image_crop, br_image_crop_resize, cv::Size(512, 512), 2);
             
            // cv::resize(tl_image_cell_guide_crop_1, tl_image_cell_guide_crop_1_resize, cv::Size(512, 512), 2);
            // cv::resize(tr_image_cell_guide_crop_1, tr_image_cell_guide_crop_1_resize, cv::Size(512, 512), 2);
            // cv::resize(bl_image_cell_guide_crop_1, bl_image_cell_guide_crop_1_resize, cv::Size(512, 512), 2);
            // cv::resize(br_image_cell_guide_crop_1, br_image_cell_guide_crop_1_resize, cv::Size(512, 512), 2);

            // cv::resize(tl_image_cell_guide_crop_2, tl_image_cell_guide_crop_2_resize, cv::Size(512, 512), 2);
            // cv::resize(tr_image_cell_guide_crop_2, tr_image_cell_guide_crop_2_resize, cv::Size(512, 512), 2);
            // cv::resize(bl_image_cell_guide_crop_2, bl_image_cell_guide_crop_2_resize, cv::Size(512, 512), 2);
            // cv::resize(br_image_cell_guide_crop_2, br_image_cell_guide_crop_2_resize, cv::Size(512, 512), 2);

            
            tl_image_crop_gpu.upload(tl_image_crop);
            tr_image_crop_gpu.upload(tr_image_crop);
            bl_image_crop_gpu.upload(bl_image_crop);
            br_image_crop_gpu.upload(br_image_crop);
            tl_image_cell_guide_crop_1_gpu.upload(tl_image_cell_guide_crop_1);
            tr_image_cell_guide_crop_1_gpu.upload(tr_image_cell_guide_crop_1);
            bl_image_cell_guide_crop_1_gpu.upload(bl_image_cell_guide_crop_1);
            br_image_cell_guide_crop_1_gpu.upload(br_image_cell_guide_crop_1);
            tl_image_cell_guide_crop_2_gpu.upload(tl_image_cell_guide_crop_2);
            tr_image_cell_guide_crop_2_gpu.upload(tr_image_cell_guide_crop_2);
            bl_image_cell_guide_crop_2_gpu.upload(bl_image_cell_guide_crop_2);
            br_image_cell_guide_crop_2_gpu.upload(br_image_cell_guide_crop_2);

            cv::cuda::resize(tl_image_crop_gpu, tl_image_crop_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(tr_image_crop_gpu, tr_image_crop_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(bl_image_crop_gpu, bl_image_crop_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(br_image_crop_gpu, br_image_crop_resize_gpu, cv::Size(512, 512), 2);
            
            cv::cuda::resize(tl_image_cell_guide_crop_1_gpu, tl_image_cell_guide_crop_1_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(tr_image_cell_guide_crop_1_gpu, tr_image_cell_guide_crop_1_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(bl_image_cell_guide_crop_1_gpu, bl_image_cell_guide_crop_1_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(br_image_cell_guide_crop_1_gpu, br_image_cell_guide_crop_1_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(tl_image_cell_guide_crop_2_gpu, tl_image_cell_guide_crop_2_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(tr_image_cell_guide_crop_2_gpu, tr_image_cell_guide_crop_2_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(bl_image_cell_guide_crop_2_gpu, bl_image_cell_guide_crop_2_resize_gpu, cv::Size(512, 512), 2);
            cv::cuda::resize(br_image_cell_guide_crop_2_gpu, br_image_cell_guide_crop_2_resize_gpu, cv::Size(512, 512), 2);

            tl_image_crop_resize_gpu.download(tl_image_crop_resize);
            tr_image_crop_resize_gpu.download(tr_image_crop_resize);
            bl_image_crop_resize_gpu.download(bl_image_crop_resize);
            br_image_crop_resize_gpu.download(br_image_crop_resize);

            tl_image_cell_guide_crop_1_resize_gpu.download(tl_image_cell_guide_crop_1_resize);
            tr_image_cell_guide_crop_1_resize_gpu.download(tr_image_cell_guide_crop_1_resize);
            bl_image_cell_guide_crop_1_resize_gpu.download(bl_image_cell_guide_crop_1_resize);
            br_image_cell_guide_crop_1_resize_gpu.download(br_image_cell_guide_crop_1_resize);
            tl_image_cell_guide_crop_2_resize_gpu.download(tl_image_cell_guide_crop_2_resize);
            tr_image_cell_guide_crop_2_resize_gpu.download(tr_image_cell_guide_crop_2_resize);
            bl_image_cell_guide_crop_2_resize_gpu.download(bl_image_cell_guide_crop_2_resize);
            br_image_cell_guide_crop_2_resize_gpu.download(br_image_cell_guide_crop_2_resize);

            tl_crop_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_crop_resize).toImageMsg();
            tr_crop_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_crop_resize).toImageMsg();
            bl_crop_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_crop_resize).toImageMsg();
            br_crop_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_crop_resize).toImageMsg();

            tl_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_cell_guide_crop_1_resize).toImageMsg();
            tr_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_cell_guide_crop_1_resize).toImageMsg();
            bl_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_cell_guide_crop_1_resize).toImageMsg();
            br_cell_guide_crop_image1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_cell_guide_crop_1_resize).toImageMsg();

            tl_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_image_cell_guide_crop_2_resize).toImageMsg();
            tr_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_image_cell_guide_crop_2_resize).toImageMsg();
            bl_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_image_cell_guide_crop_2_resize).toImageMsg();
            br_cell_guide_crop_image2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_image_cell_guide_crop_2_resize).toImageMsg();

            // crop_images.tl_image = xian_spreader_images->tl_image;
            // crop_images.tr_image = xian_spreader_images->tr_image;
            // crop_images.bl_image = xian_spreader_images->bl_image;
            // crop_images.br_image = xian_spreader_images->br_image;

            crop_images.tl_container_corner_crop_image = *tl_crop_image;
            crop_images.tr_container_corner_crop_image = *tr_crop_image;
            crop_images.bl_container_corner_crop_image = *bl_crop_image;
            crop_images.br_container_corner_crop_image = *br_crop_image;

            crop_images.tl_cell_guide_crop_image1 = *tl_cell_guide_crop_image1;
            crop_images.tr_cell_guide_crop_image1 = *tr_cell_guide_crop_image1;
            crop_images.bl_cell_guide_crop_image1 = *bl_cell_guide_crop_image1;
            crop_images.br_cell_guide_crop_image1 = *br_cell_guide_crop_image1;

            crop_images.tl_cell_guide_crop_image2 = *tl_cell_guide_crop_image2;
            crop_images.tr_cell_guide_crop_image2 = *tr_cell_guide_crop_image2;
            crop_images.bl_cell_guide_crop_image2 = *bl_cell_guide_crop_image2;
            crop_images.br_cell_guide_crop_image2 = *br_cell_guide_crop_image2;

            crop_images.container_corner_tl_x0 = tl_xy0.x;
            crop_images.container_corner_tl_y0 = tl_xy0.y;
            crop_images.container_corner_tr_x0 = tr_xy0.x;
            crop_images.container_corner_tr_y0 = tr_xy0.y;
            crop_images.container_corner_bl_x0 = bl_xy0.x;
            crop_images.container_corner_bl_y0 = bl_xy0.y;
            crop_images.container_corner_br_x0 = br_xy0.x;
            crop_images.container_corner_br_y0 = br_xy0.y;
            
            crop_images.clip1_cell_guide_tl_x = clip1_cell_guide_tl_xy0.x;
            crop_images.clip1_cell_guide_tl_y = clip1_cell_guide_tl_xy0.y;
            crop_images.clip1_cell_guide_tr_x = clip1_cell_guide_tr_xy0.x;
            crop_images.clip1_cell_guide_tr_y = clip1_cell_guide_tr_xy0.y;
            crop_images.clip1_cell_guide_bl_x = clip1_cell_guide_bl_xy0.x;
            crop_images.clip1_cell_guide_bl_y = clip1_cell_guide_bl_xy0.y;
            crop_images.clip1_cell_guide_br_x = clip1_cell_guide_br_xy0.x;
            crop_images.clip1_cell_guide_br_y = clip1_cell_guide_br_xy0.y;

            crop_images.clip2_cell_guide_tl_x = clip2_cell_guide_tl_xy0.x;
            crop_images.clip2_cell_guide_tl_y = clip2_cell_guide_tl_xy0.y;
            crop_images.clip2_cell_guide_tr_x = clip2_cell_guide_tr_xy0.x;
            crop_images.clip2_cell_guide_tr_y = clip2_cell_guide_tr_xy0.y;
            crop_images.clip2_cell_guide_bl_x = clip2_cell_guide_bl_xy0.x;
            crop_images.clip2_cell_guide_bl_y = clip2_cell_guide_bl_xy0.y;
            crop_images.clip2_cell_guide_br_x = clip2_cell_guide_br_xy0.x;
            crop_images.clip2_cell_guide_br_y = clip2_cell_guide_br_xy0.y;

            command_publisher_.publish(crop_images);


            // cv::rectangle(tl_image, tl_xy0, tl_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(tr_image, tr_xy0, tr_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(bl_image, bl_xy0, bl_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(br_image, br_xy0, br_xy1, cv::Scalar(0, 0, 255), 4); 

            // cv::rectangle(tl_image, clip1_cell_guide_tl_xy0, clip1_cell_guide_tl_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(tr_image, clip1_cell_guide_tr_xy0, clip1_cell_guide_tr_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(bl_image, clip1_cell_guide_bl_xy0, clip1_cell_guide_bl_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(br_image, clip1_cell_guide_br_xy0, clip1_cell_guide_br_xy1, cv::Scalar(0, 0, 255), 4); 

            // cv::rectangle(tl_image, clip2_cell_guide_tl_xy0, clip2_cell_guide_tl_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(tr_image, clip2_cell_guide_tr_xy0, clip2_cell_guide_tr_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(bl_image, clip2_cell_guide_bl_xy0, clip2_cell_guide_bl_xy1, cv::Scalar(0, 0, 255), 4); 
            // cv::rectangle(br_image, clip2_cell_guide_br_xy0, clip2_cell_guide_br_xy1, cv::Scalar(0, 0, 255), 4); 

            // cv::Mat src_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            // cv::Mat src_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            // cv::Mat merge_log = zpmc::zpmc_images_merge_col(src_merge_col_0, src_merge_col_1);
            // cv::resize(merge_log, merge_log, cv::Size(merge_log.cols/4, merge_log.rows/4), 2);

            // // cv::Mat crop_col_0 = zpmc::zpmc_images_merge_row(tl_image_crop, bl_image_crop);
            // // cv::Mat crop_col_1 = zpmc::zpmc_images_merge_row(tr_image_crop, br_image_crop);
            // // cv::Mat merge_crop = zpmc::zpmc_images_merge_col(crop_col_0, crop_col_1);
            // // cv::imshow("merge_crop:", merge_crop);
            // cv::imshow("images:", merge_log);
            // cv::waitKey(1);
            
            // int tl_xc0 = xian_tl_container_point_x - 100;
            // int tl_yc0 = xian_tl_container_point_y + 300;
            // cv::Point* tl_xy0 = zpmc::crop_area(tl_xc0, tl_yc0, crop_w, crop_h, src_w, src_h);
            // cv::rectangle(tl_image, *(tl_xy0+0), *(tl_xy0+1), cv::Scalar(255, 0, 0), 4);

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_container_corner_cop_process_fps", 1000.0 / timediff);
        } 
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_container_corner_cop_process_node");
    Xian_ContainerCornerCopProcess xian_container_corner_cop_process;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_container_corner_cop_process.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_ContainerCornerCopProcess::m_timer_HeartBeat_f, &xian_container_corner_cop_process);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}