#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_cell_guide_mask.h"
#include "xian_msg_pkg/xian_cell_guide_mask_resize.h"
#include "zpmc_cv_control.h"


class Xian_CellGuideMaskResize
{
    public:
        Xian_CellGuideMaskResize()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_publisher_ = nh.advertise<xian_msg_pkg::xian_cell_guide_mask_resize>("xian_cell_guide_masks_resize", 1);
            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_cell_guide_mask>("xian_cell_guide_masks", 1, &Xian_CellGuideMaskResize::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_mask_resize_heart_beat", xian_cell_guide_mask_resize_heart_beat); 
            std::cout << "xian_cell_guide_mask_resize_heart_beat: " << xian_cell_guide_mask_resize_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_cell_guide_mask_resize_heart_beat", counter);  // 自行替换
        }

    private:

        ros::Publisher command_publisher_;
        ros::Subscriber command_subscribe_;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        int xian_cell_guide_mask_resize_heart_beat = 0;
        double xian_cell_guide_mask_resize_fps = 1.0;  
        std::string timeStr;

        int tl_container_corner_cx = 0;
        int tl_container_corner_cy = 0;
        int tr_container_corner_cx = 0;
        int tr_container_corner_cy = 0;
        int bl_container_corner_cx = 0;
        int bl_container_corner_cy = 0;
        int br_container_corner_cx = 0;
        int br_container_corner_cy = 0;
        int tl_cell_guide_crop0_tl_x = 0;
        int tl_cell_guide_crop0_tl_y = 0;
        int tr_cell_guide_crop0_tl_x = 0;
        int tr_cell_guide_crop0_tl_y = 0;
        int bl_cell_guide_crop0_tl_x = 0;
        int bl_cell_guide_crop0_tl_y = 0;
        int br_cell_guide_crop0_tl_x = 0;
        int br_cell_guide_crop0_tl_y = 0;
        int tl_cell_guide_crop1_tl_x = 0;
        int tl_cell_guide_crop1_tl_y = 0;
        int tr_cell_guide_crop1_tl_x = 0;
        int tr_cell_guide_crop1_tl_y = 0;
        int bl_cell_guide_crop1_tl_x = 0;
        int bl_cell_guide_crop1_tl_y = 0;
        int br_cell_guide_crop1_tl_x = 0;
        int br_cell_guide_crop1_tl_y = 0;
        int tl_cell_guide_crop2_tl_x = 0;
        int tl_cell_guide_crop2_tl_y = 0;
        int tr_cell_guide_crop2_tl_x = 0;
        int tr_cell_guide_crop2_tl_y = 0;
        int bl_cell_guide_crop2_tl_x = 0;
        int bl_cell_guide_crop2_tl_y = 0;
        int br_cell_guide_crop2_tl_x = 0;
        int br_cell_guide_crop2_tl_y = 0;

        int crop_w = 256;
        int crop_h = 256;

        cv::Mat tl_mask_0;
        cv::Mat tr_mask_0;
        cv::Mat bl_mask_0;
        cv::Mat br_mask_0;
        cv::Mat tl_mask_1;
        cv::Mat tr_mask_1;
        cv::Mat bl_mask_1;
        cv::Mat br_mask_1;
        cv::Mat tl_mask_2;
        cv::Mat tr_mask_2;
        cv::Mat bl_mask_2;
        cv::Mat br_mask_2;

        sensor_msgs::ImagePtr tl_mask_msg0;
        sensor_msgs::ImagePtr tr_mask_msg0;
        sensor_msgs::ImagePtr bl_mask_msg0;
        sensor_msgs::ImagePtr br_mask_msg0;
        sensor_msgs::ImagePtr tl_mask_msg1;
        sensor_msgs::ImagePtr tr_mask_msg1;
        sensor_msgs::ImagePtr bl_mask_msg1;
        sensor_msgs::ImagePtr br_mask_msg1;
        sensor_msgs::ImagePtr tl_mask_msg2;
        sensor_msgs::ImagePtr tr_mask_msg2;
        sensor_msgs::ImagePtr bl_mask_msg2;
        sensor_msgs::ImagePtr br_mask_msg2;

        xian_msg_pkg::xian_cell_guide_mask_resize cell_guide_masks_resize;

        void command_callback(const xian_msg_pkg::xian_cell_guide_maskConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_mask_resize_fps", xian_cell_guide_mask_resize_fps);
            std::cout << "FPS: " << xian_cell_guide_mask_resize_fps << std::endl;

            tl_mask_0 = cv_bridge::toCvShare(data->tl_mask_0, data, "bgr8")->image; 
            tr_mask_0 = cv_bridge::toCvShare(data->tr_mask_0, data, "bgr8")->image; 
            bl_mask_0 = cv_bridge::toCvShare(data->bl_mask_0, data, "bgr8")->image;
            br_mask_0 = cv_bridge::toCvShare(data->br_mask_0, data, "bgr8")->image; 
            tl_mask_1 = cv_bridge::toCvShare(data->tl_mask_1, data, "bgr8")->image; 
            tr_mask_1 = cv_bridge::toCvShare(data->tr_mask_1, data, "bgr8")->image; 
            bl_mask_1 = cv_bridge::toCvShare(data->bl_mask_1, data, "bgr8")->image;
            br_mask_1 = cv_bridge::toCvShare(data->br_mask_1, data, "bgr8")->image; 
            tl_mask_2 = cv_bridge::toCvShare(data->tl_mask_2, data, "bgr8")->image; 
            tr_mask_2 = cv_bridge::toCvShare(data->tr_mask_2, data, "bgr8")->image; 
            bl_mask_2 = cv_bridge::toCvShare(data->bl_mask_2, data, "bgr8")->image;
            br_mask_2 = cv_bridge::toCvShare(data->br_mask_2, data, "bgr8")->image; 

            cv::resize(tl_mask_0, tl_mask_0, cv::Size(crop_w, crop_h), 2);
            cv::resize(tr_mask_0, tr_mask_0, cv::Size(crop_w, crop_h), 2);
            cv::resize(bl_mask_0, bl_mask_0, cv::Size(crop_w, crop_h), 2);
            cv::resize(br_mask_0, br_mask_0, cv::Size(crop_w, crop_h), 2);
            cv::resize(tl_mask_1, tl_mask_1, cv::Size(crop_w, crop_h), 2);
            cv::resize(tr_mask_1, tr_mask_1, cv::Size(crop_w, crop_h), 2);
            cv::resize(bl_mask_1, bl_mask_1, cv::Size(crop_w, crop_h), 2);
            cv::resize(br_mask_1, br_mask_1, cv::Size(crop_w, crop_h), 2);
            cv::resize(tl_mask_2, tl_mask_2, cv::Size(crop_w, crop_h), 2);
            cv::resize(tr_mask_2, tr_mask_2, cv::Size(crop_w, crop_h), 2);
            cv::resize(bl_mask_2, bl_mask_2, cv::Size(crop_w, crop_h), 2);
            cv::resize(br_mask_2, br_mask_2, cv::Size(crop_w, crop_h), 2);
            // printf("w: %d, h: %d \n", tl_mask_0.cols, tl_mask_0.rows);

            tl_mask_0 = zpmc::zpmc_MaskBin0_255(tl_mask_0, 80);
            tr_mask_0 = zpmc::zpmc_MaskBin0_255(tr_mask_0, 80);
            bl_mask_0 = zpmc::zpmc_MaskBin0_255(bl_mask_0, 80);
            br_mask_0 = zpmc::zpmc_MaskBin0_255(br_mask_0, 80);
            tl_mask_1 = zpmc::zpmc_MaskBin0_255(tl_mask_1, 80);
            tr_mask_1 = zpmc::zpmc_MaskBin0_255(tr_mask_1, 80);
            bl_mask_1 = zpmc::zpmc_MaskBin0_255(bl_mask_1, 80);
            br_mask_1 = zpmc::zpmc_MaskBin0_255(br_mask_1, 80);
            tl_mask_2 = zpmc::zpmc_MaskBin0_255(tl_mask_2, 80);
            tr_mask_2 = zpmc::zpmc_MaskBin0_255(tr_mask_2, 80);
            bl_mask_2 = zpmc::zpmc_MaskBin0_255(bl_mask_2, 80);
            br_mask_2 = zpmc::zpmc_MaskBin0_255(br_mask_2, 80);

            cell_guide_masks_resize.tl_image = data->tl_image;
            cell_guide_masks_resize.tr_image = data->tr_image;
            cell_guide_masks_resize.bl_image = data->bl_image;
            cell_guide_masks_resize.br_image = data->br_image;

            tl_mask_msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_mask_0).toImageMsg();
            tr_mask_msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_mask_0).toImageMsg();
            bl_mask_msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_mask_0).toImageMsg();
            br_mask_msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_mask_0).toImageMsg();
            tl_mask_msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_mask_1).toImageMsg();
            tr_mask_msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_mask_1).toImageMsg();
            bl_mask_msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_mask_1).toImageMsg();
            br_mask_msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_mask_1).toImageMsg();
            tl_mask_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_mask_2).toImageMsg();
            tr_mask_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_mask_2).toImageMsg();
            bl_mask_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_mask_2).toImageMsg();
            br_mask_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_mask_2).toImageMsg();

            cell_guide_masks_resize.tl_mask_resize_0 = *tl_mask_msg0;
            cell_guide_masks_resize.tr_mask_resize_0 = *tr_mask_msg0;
            cell_guide_masks_resize.bl_mask_resize_0 = *bl_mask_msg0;
            cell_guide_masks_resize.br_mask_resize_0 = *br_mask_msg0;
            cell_guide_masks_resize.tl_mask_resize_1 = *tl_mask_msg1;
            cell_guide_masks_resize.tr_mask_resize_1 = *tr_mask_msg1;
            cell_guide_masks_resize.bl_mask_resize_1 = *bl_mask_msg1;
            cell_guide_masks_resize.br_mask_resize_1 = *br_mask_msg1;
            cell_guide_masks_resize.tl_mask_resize_2 = *tl_mask_msg2;
            cell_guide_masks_resize.tr_mask_resize_2 = *tr_mask_msg2;
            cell_guide_masks_resize.bl_mask_resize_2 = *bl_mask_msg2;
            cell_guide_masks_resize.br_mask_resize_2 = *br_mask_msg2;

            cell_guide_masks_resize.tl_container_corner_cx = data->tl_container_corner_cx;
            cell_guide_masks_resize.tl_container_corner_cy = data->tl_container_corner_cy;
            cell_guide_masks_resize.tr_container_corner_cx = data->tr_container_corner_cx;
            cell_guide_masks_resize.tr_container_corner_cy = data->tr_container_corner_cy;
            cell_guide_masks_resize.bl_container_corner_cx = data->bl_container_corner_cx;
            cell_guide_masks_resize.bl_container_corner_cy = data->bl_container_corner_cy;
            cell_guide_masks_resize.br_container_corner_cx = data->br_container_corner_cx;
            cell_guide_masks_resize.br_container_corner_cy = data->br_container_corner_cy;

            cell_guide_masks_resize.tl_cell_guide_crop0_tl_x = data->tl_cell_guide_crop0_tl_x;
            cell_guide_masks_resize.tl_cell_guide_crop0_tl_y = data->tl_cell_guide_crop0_tl_y;
            cell_guide_masks_resize.tr_cell_guide_crop0_tl_x = data->tr_cell_guide_crop0_tl_x;
            cell_guide_masks_resize.tr_cell_guide_crop0_tl_y = data->tr_cell_guide_crop0_tl_y;
            cell_guide_masks_resize.bl_cell_guide_crop0_tl_x = data->bl_cell_guide_crop0_tl_x;
            cell_guide_masks_resize.bl_cell_guide_crop0_tl_y = data->bl_cell_guide_crop0_tl_y;
            cell_guide_masks_resize.br_cell_guide_crop0_tl_x = data->br_cell_guide_crop0_tl_x;
            cell_guide_masks_resize.br_cell_guide_crop0_tl_y = data->br_cell_guide_crop0_tl_y;
            cell_guide_masks_resize.tl_cell_guide_crop1_tl_x = data->tl_cell_guide_crop1_tl_x;
            cell_guide_masks_resize.tl_cell_guide_crop1_tl_y = data->tl_cell_guide_crop1_tl_y;
            cell_guide_masks_resize.tr_cell_guide_crop1_tl_x = data->tr_cell_guide_crop1_tl_x;
            cell_guide_masks_resize.tr_cell_guide_crop1_tl_y = data->tr_cell_guide_crop1_tl_y;
            cell_guide_masks_resize.bl_cell_guide_crop1_tl_x = data->bl_cell_guide_crop1_tl_x;
            cell_guide_masks_resize.bl_cell_guide_crop1_tl_y = data->bl_cell_guide_crop1_tl_y;
            cell_guide_masks_resize.br_cell_guide_crop1_tl_x = data->br_cell_guide_crop1_tl_x;
            cell_guide_masks_resize.br_cell_guide_crop1_tl_y = data->br_cell_guide_crop1_tl_y;
            cell_guide_masks_resize.tl_cell_guide_crop2_tl_x = data->tl_cell_guide_crop2_tl_x;
            cell_guide_masks_resize.tl_cell_guide_crop2_tl_y = data->tl_cell_guide_crop2_tl_y;
            cell_guide_masks_resize.tr_cell_guide_crop2_tl_x = data->tr_cell_guide_crop2_tl_x;
            cell_guide_masks_resize.tr_cell_guide_crop2_tl_y = data->tr_cell_guide_crop2_tl_y;
            cell_guide_masks_resize.bl_cell_guide_crop2_tl_x = data->bl_cell_guide_crop2_tl_x;
            cell_guide_masks_resize.bl_cell_guide_crop2_tl_y = data->bl_cell_guide_crop2_tl_y;
            cell_guide_masks_resize.br_cell_guide_crop2_tl_x = data->br_cell_guide_crop2_tl_x;
            cell_guide_masks_resize.br_cell_guide_crop2_tl_y = data->br_cell_guide_crop2_tl_y;

            command_publisher_.publish(cell_guide_masks_resize);

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_cell_guide_mask_resize_fps", 1000.0 / timediff);
        } 
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_cell_guide_mask_resize_node");
    Xian_CellGuideMaskResize xian_cell_guide_mask_resize;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_cell_guide_mask_resize.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_CellGuideMaskResize::m_timer_HeartBeat_f, &xian_cell_guide_mask_resize);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}