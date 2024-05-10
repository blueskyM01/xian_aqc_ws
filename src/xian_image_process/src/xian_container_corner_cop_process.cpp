#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "xian_msg_pkg/xian_container_corner_crop_image_msg.h"
#include "zpmc_cv_control.h"


class Xian_ContainerCornerCopProcess
{
    public:
        Xian_ContainerCornerCopProcess()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_publisher_ = nh.advertise<xian_msg_pkg::xian_container_corner_crop_image_msg>("xian_container_corner_crop_images", 1);
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

        int crop_w = 256;
        int crop_h = 256;

 
        cv::Mat tl_image, tr_image, bl_image, br_image;
        // cv::Mat tl_preprocessed_image, tr_preprocessed_image, bl_preprocessed_image, br_preprocessed_image;
        // sensor_msgs::ImagePtr zpmc_tl_preprocess_image, zpmc_tr_preprocess_image, zpmc_bl_preprocess_image, zpmc_br_preprocess_image;
        // zpmc_unloading_interface_pkg::ZpmcLockHolePreprocessImages zpmc_corner_line_preprocess_images;


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

            tl_image = cv_bridge::toCvShare(xian_spreader_images->tl_image, xian_spreader_images, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(xian_spreader_images->tr_image, xian_spreader_images, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(xian_spreader_images->bl_image, xian_spreader_images, "bgr8")->image;
            br_image = cv_bridge::toCvShare(xian_spreader_images->br_image, xian_spreader_images, "bgr8")->image; 
            int src_w = tl_image.cols;
            int src_h = tl_image.rows;

            cv::Point* tl_xy = zpmc::crop_area(xian_tl_container_point_x, xian_tl_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::rectangle(tl_image, *(tl_xy+0), *(tl_xy+1), cv::Scalar(0, 0, 255), 4); 

            cv::Point* tr_xy = zpmc::crop_area(xian_tr_container_point_x, xian_tr_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::rectangle(tr_image, *(tr_xy+0), *(tr_xy+1), cv::Scalar(0, 0, 255), 4); 

            cv::Point* bl_xy = zpmc::crop_area(xian_bl_container_point_x, xian_bl_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::rectangle(bl_image, *(bl_xy+0), *(bl_xy+1), cv::Scalar(0, 0, 255), 4); 

            cv::Point* br_xy = zpmc::crop_area(xian_br_container_point_x, xian_br_container_point_y, crop_w, crop_h, src_w, src_h);
            cv::rectangle(br_image, *(br_xy+0), *(br_xy+1), cv::Scalar(0, 0, 255), 4); 



            int tl_xc0 = xian_tl_container_point_x - 100;
            int tl_yc0 = xian_tl_container_point_y + 300;
            cv::Point* tl_xy0 = zpmc::crop_area(tl_xc0, tl_yc0, crop_w, crop_h, src_w, src_h);
            cv::rectangle(tl_image, *(tl_xy0+0), *(tl_xy0+1), cv::Scalar(255, 0, 0), 4);



            cv::Mat src_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            cv::Mat src_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            cv::Mat merge_log = zpmc::zpmc_images_merge_col(src_merge_col_0, src_merge_col_1);
            cv::resize(merge_log, merge_log, cv::Size(merge_log.cols/3, merge_log.rows/3), 2);

            cv::imshow("images:", merge_log);
            cv::waitKey(1);

            // cv::resize(tl_image, tl_preprocessed_image, cv::Size(550, 550), 2);
            // cv::resize(tr_image, tr_preprocessed_image, cv::Size(550, 550), 2);
            // cv::resize(bl_image, bl_preprocessed_image, cv::Size(550, 550), 2);
            // cv::resize(br_image, br_preprocessed_image, cv::Size(550, 550), 2);


            // zpmc_tl_preprocess_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_preprocessed_image).toImageMsg();
            // zpmc_tr_preprocess_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_preprocessed_image).toImageMsg();
            // zpmc_bl_preprocess_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_preprocessed_image).toImageMsg();
            // zpmc_br_preprocess_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_preprocessed_image).toImageMsg();

            // zpmc_corner_line_preprocess_images.tl_image = xian_spreader_images->tl_image;
            // zpmc_corner_line_preprocess_images.tr_image = xian_spreader_images->tr_image;
            // zpmc_corner_line_preprocess_images.bl_image = xian_spreader_images->bl_image;
            // zpmc_corner_line_preprocess_images.br_image = xian_spreader_images->br_image;

            // zpmc_corner_line_preprocess_images.tl_preprocess_image = *zpmc_tl_preprocess_image;
            // zpmc_corner_line_preprocess_images.tr_preprocess_image = *zpmc_tr_preprocess_image;
            // zpmc_corner_line_preprocess_images.bl_preprocess_image = *zpmc_bl_preprocess_image;
            // zpmc_corner_line_preprocess_images.br_preprocess_image = *zpmc_br_preprocess_image;

            // command_publisher_.publish(zpmc_corner_line_preprocess_images);

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