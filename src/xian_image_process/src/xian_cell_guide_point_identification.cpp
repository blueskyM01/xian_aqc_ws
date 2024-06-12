#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_cell_guide_mask_resize.h"
#include "xian_msg_pkg/xian_cell_guide_point_identification.h"
#include "zpmc_cv_control.h"


class Xian_CellGuidePointIdentification
{
    public:
        Xian_CellGuidePointIdentification()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_publisher_ = nh.advertise<xian_msg_pkg::xian_cell_guide_point_identification>("xian_cell_guide_points", 1);
            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_cell_guide_mask_resize>("xian_cell_guide_masks_resize", 1, &Xian_CellGuidePointIdentification::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_point_identification_heart_beat", xian_cell_guide_point_identification_heart_beat); 
            std::cout << "xian_cell_guide_point_identification_heart_beat: " << xian_cell_guide_point_identification_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_cell_guide_point_identification_heart_beat", counter);  // 自行替换
        }

    private:

        ros::Publisher command_publisher_;
        ros::Subscriber command_subscribe_;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        int xian_cell_guide_point_identification_heart_beat = 0;
        double xian_cell_guide_point_identification_fps = 1.0;  
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
        int L_Type_request;
        double scale_lower = 0;
        double scale_upper = 0;
        double point_w_a = 0;
        double point_w_b = 0;
        int area_threshold = 0;

        cv::Mat tl_mask_resize_0;
        cv::Mat tr_mask_resize_0;
        cv::Mat bl_mask_resize_0;
        cv::Mat br_mask_resize_0;
        cv::Mat tl_mask_resize_1;
        cv::Mat tr_mask_resize_1;
        cv::Mat bl_mask_resize_1;
        cv::Mat br_mask_resize_1;
        cv::Mat tl_mask_resize_2;
        cv::Mat tr_mask_resize_2;
        cv::Mat bl_mask_resize_2;
        cv::Mat br_mask_resize_2;

        cv::Mat tl_mask_resize_0_show;
        cv::Mat tr_mask_resize_0_show;
        cv::Mat bl_mask_resize_0_show;
        cv::Mat br_mask_resize_0_show;
        cv::Mat tl_mask_resize_1_show;
        cv::Mat tr_mask_resize_1_show;
        cv::Mat bl_mask_resize_1_show;
        cv::Mat br_mask_resize_1_show;
        cv::Mat tl_mask_resize_2_show;
        cv::Mat tr_mask_resize_2_show;
        cv::Mat bl_mask_resize_2_show;
        cv::Mat br_mask_resize_2_show;

        sensor_msgs::ImagePtr tl_mask_resize_0_show_msg;
        sensor_msgs::ImagePtr tr_mask_resize_0_show_msg;
        sensor_msgs::ImagePtr bl_mask_resize_0_show_msg;
        sensor_msgs::ImagePtr br_mask_resize_0_show_msg;
        sensor_msgs::ImagePtr tl_mask_resize_1_show_msg;
        sensor_msgs::ImagePtr tr_mask_resize_1_show_msg;
        sensor_msgs::ImagePtr bl_mask_resize_1_show_msg;
        sensor_msgs::ImagePtr br_mask_resize_1_show_msg;
        sensor_msgs::ImagePtr tl_mask_resize_2_show_msg;
        sensor_msgs::ImagePtr tr_mask_resize_2_show_msg;
        sensor_msgs::ImagePtr bl_mask_resize_2_show_msg;
        sensor_msgs::ImagePtr br_mask_resize_2_show_msg;
        xian_msg_pkg::xian_cell_guide_point_identification cell_guide_points;

        void command_callback(const xian_msg_pkg::xian_cell_guide_mask_resizeConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_point_identification_fps", xian_cell_guide_point_identification_fps);
            std::cout << "FPS: " << xian_cell_guide_point_identification_fps << std::endl;

            ros::param::get("/xian_aqc_dynamic_parameters_server/scale_lower", scale_lower);
            ros::param::get("/xian_aqc_dynamic_parameters_server/scale_upper", scale_upper);
            ros::param::get("/xian_aqc_dynamic_parameters_server/point_w_a", point_w_a);
            ros::param::get("/xian_aqc_dynamic_parameters_server/point_w_b", point_w_b);
            ros::param::get("/xian_aqc_dynamic_parameters_server/area_threshold", area_threshold);

            tl_mask_resize_0 = cv_bridge::toCvShare(data->tl_mask_resize_0, data, "bgr8")->image; 
            tr_mask_resize_0 = cv_bridge::toCvShare(data->tr_mask_resize_0, data, "bgr8")->image; 
            bl_mask_resize_0 = cv_bridge::toCvShare(data->bl_mask_resize_0, data, "bgr8")->image;
            br_mask_resize_0 = cv_bridge::toCvShare(data->br_mask_resize_0, data, "bgr8")->image; 
            tl_mask_resize_1 = cv_bridge::toCvShare(data->tl_mask_resize_1, data, "bgr8")->image; 
            tr_mask_resize_1 = cv_bridge::toCvShare(data->tr_mask_resize_1, data, "bgr8")->image; 
            bl_mask_resize_1 = cv_bridge::toCvShare(data->bl_mask_resize_1, data, "bgr8")->image;
            br_mask_resize_1 = cv_bridge::toCvShare(data->br_mask_resize_1, data, "bgr8")->image; 
            tl_mask_resize_2 = cv_bridge::toCvShare(data->tl_mask_resize_2, data, "bgr8")->image; 
            tr_mask_resize_2 = cv_bridge::toCvShare(data->tr_mask_resize_2, data, "bgr8")->image; 
            bl_mask_resize_2 = cv_bridge::toCvShare(data->bl_mask_resize_2, data, "bgr8")->image;
            br_mask_resize_2 = cv_bridge::toCvShare(data->br_mask_resize_2, data, "bgr8")->image; 

            tl_cell_guide_crop0_tl_x = data->tl_cell_guide_crop0_tl_x;
            tl_cell_guide_crop0_tl_y = data->tl_cell_guide_crop0_tl_y;
            tr_cell_guide_crop0_tl_x = data->tr_cell_guide_crop0_tl_x;
            tr_cell_guide_crop0_tl_y = data->tr_cell_guide_crop0_tl_y;
            bl_cell_guide_crop0_tl_x = data->bl_cell_guide_crop0_tl_x;
            bl_cell_guide_crop0_tl_y = data->bl_cell_guide_crop0_tl_y;
            br_cell_guide_crop0_tl_x = data->br_cell_guide_crop0_tl_x;
            br_cell_guide_crop0_tl_y = data->br_cell_guide_crop0_tl_y;

            tl_cell_guide_crop1_tl_x = data->tl_cell_guide_crop1_tl_x;
            tl_cell_guide_crop1_tl_y = data->tl_cell_guide_crop1_tl_y;
            tr_cell_guide_crop1_tl_x = data->tr_cell_guide_crop1_tl_x;
            tr_cell_guide_crop1_tl_y = data->tr_cell_guide_crop1_tl_y;
            bl_cell_guide_crop1_tl_x = data->bl_cell_guide_crop1_tl_x;
            bl_cell_guide_crop1_tl_y = data->bl_cell_guide_crop1_tl_y;
            br_cell_guide_crop1_tl_x = data->br_cell_guide_crop1_tl_x;
            br_cell_guide_crop1_tl_y = data->br_cell_guide_crop1_tl_y;

            tl_cell_guide_crop2_tl_x = data->tl_cell_guide_crop2_tl_x;
            tl_cell_guide_crop2_tl_y = data->tl_cell_guide_crop2_tl_y;
            tr_cell_guide_crop2_tl_x = data->tr_cell_guide_crop2_tl_x;
            tr_cell_guide_crop2_tl_y = data->tr_cell_guide_crop2_tl_y;
            bl_cell_guide_crop2_tl_x = data->bl_cell_guide_crop2_tl_x;
            bl_cell_guide_crop2_tl_y = data->bl_cell_guide_crop2_tl_y;
            br_cell_guide_crop2_tl_x = data->br_cell_guide_crop2_tl_x;
            br_cell_guide_crop2_tl_y = data->br_cell_guide_crop2_tl_y;

            // -----------------------------crop0-----------------------------
            std::vector<std::vector<double>>* target_results_set0 = xian_get_cell_guide_results(tl_mask_resize_0, tr_mask_resize_0, 
                                                                                                bl_mask_resize_0, br_mask_resize_0,
                                                                                                scale_lower, scale_upper, 
                                                                                                point_w_a, point_w_b, 
                                                                                                area_threshold, 
                                                                                                crop_w, crop_h);
            
            int* target_points_set0 = xian_get_cell_guide_points(target_results_set0,
                                                                 tl_cell_guide_crop0_tl_x, tl_cell_guide_crop0_tl_y,
                                                                 tr_cell_guide_crop0_tl_x, tr_cell_guide_crop0_tl_y,
                                                                 bl_cell_guide_crop0_tl_x, bl_cell_guide_crop0_tl_y,
                                                                 br_cell_guide_crop0_tl_x, br_cell_guide_crop0_tl_y);

            cell_guide_points.tl_cell_guide0_cx = *(target_points_set0+0);
            cell_guide_points.tl_cell_guide0_cy = *(target_points_set0+1);
            cell_guide_points.tr_cell_guide0_cx = *(target_points_set0+2);
            cell_guide_points.tr_cell_guide0_cy = *(target_points_set0+3);
            cell_guide_points.bl_cell_guide0_cx = *(target_points_set0+4);
            cell_guide_points.bl_cell_guide0_cy = *(target_points_set0+5);
            cell_guide_points.br_cell_guide0_cx = *(target_points_set0+6);
            cell_guide_points.br_cell_guide0_cy = *(target_points_set0+7);

            // -----------------------------crop0 result show-----------------------------
            std::vector<std::vector<double>> target_results_set0_tl = *(target_results_set0+0);
            std::vector<std::vector<double>> target_results_set0_tr = *(target_results_set0+1);
            std::vector<std::vector<double>> target_results_set0_bl = *(target_results_set0+2);
            std::vector<std::vector<double>> target_results_set0_br = *(target_results_set0+3);
            tl_mask_resize_0_show = zpmc::zpmc_show_L_marker_mask_result(tl_mask_resize_0, target_results_set0_tl, 0);
            tr_mask_resize_0_show = zpmc::zpmc_show_L_marker_mask_result(tr_mask_resize_0, target_results_set0_tr, 1);
            bl_mask_resize_0_show = zpmc::zpmc_show_L_marker_mask_result(bl_mask_resize_0, target_results_set0_bl, 2);
            br_mask_resize_0_show = zpmc::zpmc_show_L_marker_mask_result(br_mask_resize_0, target_results_set0_br, 3);

            // -----------------------------crop1-----------------------------
            std::vector<std::vector<double>>* target_results_set1 = xian_get_cell_guide_results(tl_mask_resize_1, tr_mask_resize_1, 
                                                                                                bl_mask_resize_1, br_mask_resize_1,
                                                                                                scale_lower, scale_upper, 
                                                                                                point_w_a, point_w_b, 
                                                                                                area_threshold, 
                                                                                                crop_w, crop_h);
            int* target_points_set1 = xian_get_cell_guide_points(target_results_set1,
                                                                 tl_cell_guide_crop1_tl_x, tl_cell_guide_crop1_tl_y,
                                                                 tr_cell_guide_crop1_tl_x, tr_cell_guide_crop1_tl_y,
                                                                 bl_cell_guide_crop1_tl_x, bl_cell_guide_crop1_tl_y,
                                                                 br_cell_guide_crop1_tl_x, br_cell_guide_crop1_tl_y);
            cell_guide_points.tl_cell_guide1_cx = *(target_points_set1+0);
            cell_guide_points.tl_cell_guide1_cy = *(target_points_set1+1);
            cell_guide_points.tr_cell_guide1_cx = *(target_points_set1+2);
            cell_guide_points.tr_cell_guide1_cy = *(target_points_set1+3);
            cell_guide_points.bl_cell_guide1_cx = *(target_points_set1+4);
            cell_guide_points.bl_cell_guide1_cy = *(target_points_set1+5);
            cell_guide_points.br_cell_guide1_cx = *(target_points_set1+6);
            cell_guide_points.br_cell_guide1_cy = *(target_points_set1+7);
            // -----------------------------crop1 result show-----------------------------
            std::vector<std::vector<double>> target_results_set1_tl = *(target_results_set1+0);
            std::vector<std::vector<double>> target_results_set1_tr = *(target_results_set1+1);
            std::vector<std::vector<double>> target_results_set1_bl = *(target_results_set1+2);
            std::vector<std::vector<double>> target_results_set1_br = *(target_results_set1+3);
            tl_mask_resize_1_show = zpmc::zpmc_show_L_marker_mask_result(tl_mask_resize_1, target_results_set1_tl, 0);
            tr_mask_resize_1_show = zpmc::zpmc_show_L_marker_mask_result(tr_mask_resize_1, target_results_set1_tr, 1);
            bl_mask_resize_1_show = zpmc::zpmc_show_L_marker_mask_result(bl_mask_resize_1, target_results_set1_bl, 2);
            br_mask_resize_1_show = zpmc::zpmc_show_L_marker_mask_result(br_mask_resize_1, target_results_set1_br, 3);
            // -----------------------------crop2-----------------------------
            std::vector<std::vector<double>>* target_results_set2 = xian_get_cell_guide_results(tl_mask_resize_2, tr_mask_resize_2, 
                                                                                                bl_mask_resize_2, br_mask_resize_2,
                                                                                                scale_lower, scale_upper, 
                                                                                                point_w_a, point_w_b, 
                                                                                                area_threshold, 
                                                                                                crop_w, crop_h);
            int* target_points_set2 = xian_get_cell_guide_points(target_results_set2,
                                                                 tl_cell_guide_crop2_tl_x, tl_cell_guide_crop2_tl_y,
                                                                 tr_cell_guide_crop2_tl_x, tr_cell_guide_crop2_tl_y,
                                                                 bl_cell_guide_crop2_tl_x, bl_cell_guide_crop2_tl_y,
                                                                 br_cell_guide_crop2_tl_x, br_cell_guide_crop2_tl_y);
            cell_guide_points.tl_cell_guide2_cx = *(target_points_set2+0);
            cell_guide_points.tl_cell_guide2_cy = *(target_points_set2+1);
            cell_guide_points.tr_cell_guide2_cx = *(target_points_set2+2);
            cell_guide_points.tr_cell_guide2_cy = *(target_points_set2+3);
            cell_guide_points.bl_cell_guide2_cx = *(target_points_set2+4);
            cell_guide_points.bl_cell_guide2_cy = *(target_points_set2+5);
            cell_guide_points.br_cell_guide2_cx = *(target_points_set2+6);
            cell_guide_points.br_cell_guide2_cy = *(target_points_set2+7);
            // -----------------------------crop2 result show-----------------------------
            std::vector<std::vector<double>> target_results_set2_tl = *(target_results_set2+0);
            std::vector<std::vector<double>> target_results_set2_tr = *(target_results_set2+1);
            std::vector<std::vector<double>> target_results_set2_bl = *(target_results_set2+2);
            std::vector<std::vector<double>> target_results_set2_br = *(target_results_set2+3);
            tl_mask_resize_2_show = zpmc::zpmc_show_L_marker_mask_result(tl_mask_resize_2, target_results_set2_tl, 0);
            tr_mask_resize_2_show = zpmc::zpmc_show_L_marker_mask_result(tr_mask_resize_2, target_results_set2_tr, 1);
            bl_mask_resize_2_show = zpmc::zpmc_show_L_marker_mask_result(bl_mask_resize_2, target_results_set2_bl, 2);
            br_mask_resize_2_show = zpmc::zpmc_show_L_marker_mask_result(br_mask_resize_2, target_results_set2_br, 3);

            tl_mask_resize_0_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_mask_resize_0_show).toImageMsg();
            tr_mask_resize_0_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_mask_resize_0_show).toImageMsg();
            bl_mask_resize_0_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_mask_resize_0_show).toImageMsg();
            br_mask_resize_0_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_mask_resize_0_show).toImageMsg();
            tl_mask_resize_1_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_mask_resize_1_show).toImageMsg();
            tr_mask_resize_1_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_mask_resize_1_show).toImageMsg();
            bl_mask_resize_1_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_mask_resize_1_show).toImageMsg();
            br_mask_resize_1_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_mask_resize_1_show).toImageMsg();
            tl_mask_resize_2_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_mask_resize_2_show).toImageMsg();
            tr_mask_resize_2_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tr_mask_resize_2_show).toImageMsg();
            bl_mask_resize_2_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bl_mask_resize_2_show).toImageMsg();
            br_mask_resize_2_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_mask_resize_2_show).toImageMsg();

            cell_guide_points.tl_mask_resize_0_show = *tl_mask_resize_0_show_msg;
            cell_guide_points.tr_mask_resize_0_show = *tr_mask_resize_0_show_msg;
            cell_guide_points.bl_mask_resize_0_show = *bl_mask_resize_0_show_msg;
            cell_guide_points.br_mask_resize_0_show = *br_mask_resize_0_show_msg;
            cell_guide_points.tl_mask_resize_1_show = *tl_mask_resize_1_show_msg;
            cell_guide_points.tr_mask_resize_1_show = *tr_mask_resize_1_show_msg;
            cell_guide_points.bl_mask_resize_1_show = *bl_mask_resize_1_show_msg;
            cell_guide_points.br_mask_resize_1_show = *br_mask_resize_1_show_msg;
            cell_guide_points.tl_mask_resize_2_show = *tl_mask_resize_2_show_msg;
            cell_guide_points.tr_mask_resize_2_show = *tr_mask_resize_2_show_msg;
            cell_guide_points.bl_mask_resize_2_show = *bl_mask_resize_2_show_msg;
            cell_guide_points.br_mask_resize_2_show = *br_mask_resize_2_show_msg;

            cell_guide_points.tl_image = data->tl_image;
            cell_guide_points.tr_image = data->tr_image;
            cell_guide_points.bl_image = data->bl_image;
            cell_guide_points.br_image = data->br_image;

            cell_guide_points.tl_container_corner_cx = data->tl_container_corner_cx;
            cell_guide_points.tl_container_corner_cy = data->tl_container_corner_cy;
            cell_guide_points.tr_container_corner_cx = data->tr_container_corner_cx;
            cell_guide_points.tr_container_corner_cy = data->tr_container_corner_cy;
            cell_guide_points.bl_container_corner_cx = data->bl_container_corner_cx;
            cell_guide_points.bl_container_corner_cy = data->bl_container_corner_cy;
            cell_guide_points.br_container_corner_cx = data->br_container_corner_cx;
            cell_guide_points.br_container_corner_cy = data->br_container_corner_cy;

            cell_guide_points.tl_cell_guide_crop0_tl_x = data->tl_cell_guide_crop0_tl_x;
            cell_guide_points.tl_cell_guide_crop0_tl_y = data->tl_cell_guide_crop0_tl_y;
            cell_guide_points.tr_cell_guide_crop0_tl_x = data->tr_cell_guide_crop0_tl_x;
            cell_guide_points.tr_cell_guide_crop0_tl_y = data->tr_cell_guide_crop0_tl_y;
            cell_guide_points.bl_cell_guide_crop0_tl_x = data->bl_cell_guide_crop0_tl_x;
            cell_guide_points.bl_cell_guide_crop0_tl_y = data->bl_cell_guide_crop0_tl_y;
            cell_guide_points.br_cell_guide_crop0_tl_x = data->br_cell_guide_crop0_tl_x;
            cell_guide_points.br_cell_guide_crop0_tl_y = data->br_cell_guide_crop0_tl_y;
            cell_guide_points.tl_cell_guide_crop1_tl_x = data->tl_cell_guide_crop1_tl_x;
            cell_guide_points.tl_cell_guide_crop1_tl_y = data->tl_cell_guide_crop1_tl_y;
            cell_guide_points.tr_cell_guide_crop1_tl_x = data->tr_cell_guide_crop1_tl_x;
            cell_guide_points.tr_cell_guide_crop1_tl_y = data->tr_cell_guide_crop1_tl_y;
            cell_guide_points.bl_cell_guide_crop1_tl_x = data->bl_cell_guide_crop1_tl_x;
            cell_guide_points.bl_cell_guide_crop1_tl_y = data->bl_cell_guide_crop1_tl_y;
            cell_guide_points.br_cell_guide_crop1_tl_x = data->br_cell_guide_crop1_tl_x;
            cell_guide_points.br_cell_guide_crop1_tl_y = data->br_cell_guide_crop1_tl_y;
            cell_guide_points.tl_cell_guide_crop2_tl_x = data->tl_cell_guide_crop2_tl_x;
            cell_guide_points.tl_cell_guide_crop2_tl_y = data->tl_cell_guide_crop2_tl_y;
            cell_guide_points.tr_cell_guide_crop2_tl_x = data->tr_cell_guide_crop2_tl_x;
            cell_guide_points.tr_cell_guide_crop2_tl_y = data->tr_cell_guide_crop2_tl_y;
            cell_guide_points.bl_cell_guide_crop2_tl_x = data->bl_cell_guide_crop2_tl_x;
            cell_guide_points.bl_cell_guide_crop2_tl_y = data->bl_cell_guide_crop2_tl_y;
            cell_guide_points.br_cell_guide_crop2_tl_x = data->br_cell_guide_crop2_tl_x;
            cell_guide_points.br_cell_guide_crop2_tl_y = data->br_cell_guide_crop2_tl_y;
            
            command_publisher_.publish(cell_guide_points);

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_cell_guide_point_identification_fps", 1000.0 / timediff);
        } 

        int* xian_get_cell_guide_points(std::vector<std::vector<double>>* target_results_set,
                                        int tl_cell_guide_crop_tl_x, int tl_cell_guide_crop_tl_y,
                                        int tr_cell_guide_crop_tl_x, int tr_cell_guide_crop_tl_y,
                                        int bl_cell_guide_crop_tl_x, int bl_cell_guide_crop_tl_y,
                                        int br_cell_guide_crop_tl_x, int br_cell_guide_crop_tl_y)
        {
            static int target_points_set[8];
            double tl_corner_x = 0;
            double tl_corner_y = 0;
            double tr_corner_x = 0;
            double tr_corner_y = 0;
            double bl_corner_x = 0;
            double bl_corner_y = 0;
            double br_corner_x = 0;
            double br_corner_y = 0;
            if(tl_cell_guide_crop_tl_x==0 && tl_cell_guide_crop_tl_y==0)
            {
                tl_corner_x = -1;
                tl_corner_y = -1;
            }
            else
            {
                std::vector<std::vector<double>> target_points_set_tl = *(target_results_set+0);
                tl_corner_x = target_points_set_tl[target_points_set_tl.size()-1][0];
                tl_corner_y = target_points_set_tl[target_points_set_tl.size()-1][1];
            }
            
            if(tr_cell_guide_crop_tl_x==0 && tr_cell_guide_crop_tl_y==0)
            {
                tr_corner_x = -1;
                tr_corner_y = -1;
            }
            else
            {
                std::vector<std::vector<double>> target_points_set_tr = *(target_results_set+1);
                tr_corner_x = target_points_set_tr[target_points_set_tr.size()-1][0];
                tr_corner_y = target_points_set_tr[target_points_set_tr.size()-1][1];
            }

            if(bl_cell_guide_crop_tl_x==0 && bl_cell_guide_crop_tl_y==0)
            {
                bl_corner_x = -1;
                bl_corner_y = -1;
            }
            else
            {
                std::vector<std::vector<double>> target_points_set_bl = *(target_results_set+2);
                bl_corner_x = target_points_set_bl[target_points_set_bl.size()-1][0];
                bl_corner_y = target_points_set_bl[target_points_set_bl.size()-1][1];
            }

            if(br_cell_guide_crop_tl_x==0 && br_cell_guide_crop_tl_y==0)
            {
                br_corner_x = -1;
                br_corner_y = -1;
            }
            else
            {
                std::vector<std::vector<double>> target_points_set_br = *(target_results_set+3);
                br_corner_x = target_points_set_br[target_points_set_br.size()-1][0];
                br_corner_y = target_points_set_br[target_points_set_br.size()-1][1];
            }
            target_points_set[0] = (int)tl_corner_x;
            target_points_set[1] = (int)tl_corner_y;
            target_points_set[2] = (int)tr_corner_x;
            target_points_set[3] = (int)tr_corner_y;
            target_points_set[4] = (int)bl_corner_x;
            target_points_set[5] = (int)bl_corner_y;
            target_points_set[6] = (int)br_corner_x;
            target_points_set[7] = (int)br_corner_y;
            return target_points_set;
        }

        std::vector<std::vector<double>>* xian_get_cell_guide_results(cv::Mat tl_mask_resize, cv::Mat tr_mask_resize, 
                                                                      cv::Mat bl_mask_resize, cv::Mat br_mask_resize,
                                                                      double scale_lower, double scale_upper, 
                                                                      double point_w_a, double point_w_b, 
                                                                      int area_threshold, 
                                                                      int crop_w, int crop_h)
        {
            static std::vector<std::vector<double>> target_result_set[4];

            L_Type_request = 0;
            std::vector<std::vector<double>> target_points_set_tl = zpmc::zpmc_marker_target_points_get(tl_mask_resize, L_Type_request, 
                                                                                                        scale_lower, scale_upper, 
                                                                                                        point_w_a, point_w_b, area_threshold, 
                                                                                                        crop_w-1, crop_h-1, 
                                                                                                        crop_w+120);
            
            L_Type_request = 1;
            std::vector<std::vector<double>> target_points_set_tr = zpmc::zpmc_marker_target_points_get(tr_mask_resize, L_Type_request, 
                                                                                                        scale_lower, scale_upper, 
                                                                                                        point_w_a, point_w_b, area_threshold, 
                                                                                                        0, crop_h-1, 
                                                                                                        crop_w+120);
            
            L_Type_request = 2;
            std::vector<std::vector<double>> target_points_set_bl = zpmc::zpmc_marker_target_points_get(bl_mask_resize, L_Type_request, 
                                                                                                        scale_lower, scale_upper, 
                                                                                                        point_w_a, point_w_b, area_threshold, 
                                                                                                        crop_w-1, 0, 
                                                                                                        crop_w+120);

            L_Type_request = 3;
            std::vector<std::vector<double>> target_points_set_br = zpmc::zpmc_marker_target_points_get(br_mask_resize, L_Type_request, 
                                                                                                        scale_lower, scale_upper, 
                                                                                                        point_w_a, point_w_b, area_threshold, 
                                                                                                        0, 0, 
                                                                                                        crop_w+120);
            target_result_set[0] = target_points_set_tl;
            target_result_set[1] = target_points_set_tr;
            target_result_set[2] = target_points_set_bl;
            target_result_set[3] = target_points_set_br;

            return target_result_set;
        }
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_cell_guide_point_identification_node");
    Xian_CellGuidePointIdentification xian_cell_guide_point_identification;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_cell_guide_point_identification.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_CellGuidePointIdentification::m_timer_HeartBeat_f, &xian_cell_guide_point_identification);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}