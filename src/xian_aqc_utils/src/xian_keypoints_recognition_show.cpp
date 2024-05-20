#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_keypoints.h"
#include "zpmc_cv_control.h"


class Xian_KeypointsRecognitionShow
{
    public:
        Xian_KeypointsRecognitionShow()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_keypoints>("xian_aqc_keypoints", 1, &Xian_KeypointsRecognitionShow::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_keypoints_recognition_show_heart_beat: " << counter << std::endl;
        }

    private:
        ros::Subscriber command_subscribe_;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        std::string timeStr;

        int xian_tl_container_point_x = 0;
        int xian_tl_container_point_y = 0;
        int xian_tr_container_point_x = 0;
        int xian_tr_container_point_y = 0;
        int xian_bl_container_point_x = 0;
        int xian_bl_container_point_y = 0;
        int xian_br_container_point_x = 0;
        int xian_br_container_point_y = 0;

        int tl_container_corner_x = 0;
        int tl_container_corner_y = 0;
        int tr_container_corner_x = 0;
        int tr_container_corner_y = 0;
        int bl_container_corner_x = 0;
        int bl_container_corner_y = 0;
        int br_container_corner_x = 0;
        int br_container_corner_y = 0;
        int tl_cell_guide_crop0_x = 0;
        int tl_cell_guide_crop0_y = 0;
        int tr_cell_guide_crop0_x = 0;
        int tr_cell_guide_crop0_y = 0;
        int bl_cell_guide_crop0_x = 0;
        int bl_cell_guide_crop0_y = 0;
        int br_cell_guide_crop0_x = 0;
        int br_cell_guide_crop0_y = 0;
        int tl_cell_guide_crop1_x = 0;
        int tl_cell_guide_crop1_y = 0;
        int tr_cell_guide_crop1_x = 0;
        int tr_cell_guide_crop1_y = 0;
        int bl_cell_guide_crop1_x = 0;
        int bl_cell_guide_crop1_y = 0;
        int br_cell_guide_crop1_x = 0;
        int br_cell_guide_crop1_y = 0;
        int tl_cell_guide_crop2_x = 0;
        int tl_cell_guide_crop2_y = 0;
        int tr_cell_guide_crop2_x = 0;
        int tr_cell_guide_crop2_y = 0;
        int bl_cell_guide_crop2_x = 0;
        int bl_cell_guide_crop2_y = 0;
        int br_cell_guide_crop2_x = 0;
        int br_cell_guide_crop2_y = 0;

        cv::Point tl_container_corner_calibrated = cv::Point(0,0);
        cv::Point tr_container_corner_calibrated = cv::Point(0,0);
        cv::Point bl_container_corner_calibrated = cv::Point(0,0);
        cv::Point br_container_corner_calibrated = cv::Point(0,0);

        cv::Point tl_container_corner_identified = cv::Point(0,0);
        cv::Point tr_container_corner_identified = cv::Point(0,0);
        cv::Point bl_container_corner_identified = cv::Point(0,0);
        cv::Point br_container_corner_identified = cv::Point(0,0);

        cv::Point tl_cell_guide_crop0 = cv::Point(0,0);
        cv::Point tr_cell_guide_crop0 = cv::Point(0,0);
        cv::Point bl_cell_guide_crop0 = cv::Point(0,0);
        cv::Point br_cell_guide_crop0 = cv::Point(0,0);

        cv::Point tl_cell_guide_crop1 = cv::Point(0,0);
        cv::Point tr_cell_guide_crop1 = cv::Point(0,0);
        cv::Point bl_cell_guide_crop1 = cv::Point(0,0);
        cv::Point br_cell_guide_crop1 = cv::Point(0,0);

        cv::Point tl_cell_guide_crop2 = cv::Point(0,0);
        cv::Point tr_cell_guide_crop2 = cv::Point(0,0);
        cv::Point bl_cell_guide_crop2 = cv::Point(0,0);
        cv::Point br_cell_guide_crop2 = cv::Point(0,0);
 
        cv::Mat tl_image, tr_image, bl_image, br_image;

        void command_callback(const xian_msg_pkg::xian_keypointsConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();

            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_x", xian_tl_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_y", xian_tl_container_point_y);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_x", xian_tr_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_y", xian_tr_container_point_y);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_x", xian_bl_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_y", xian_bl_container_point_y);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_br_container_point_x", xian_br_container_point_x);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_br_container_point_y", xian_br_container_point_y);

            tl_image = cv_bridge::toCvShare(data->tl_image, data, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(data->tr_image, data, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(data->bl_image, data, "bgr8")->image;
            br_image = cv_bridge::toCvShare(data->br_image, data, "bgr8")->image;

            tl_container_corner_x = data->tl_container_corner_x;
            tl_container_corner_y = data->tl_container_corner_y;
            tr_container_corner_x = data->tr_container_corner_x;
            tr_container_corner_y = data->tr_container_corner_y;
            bl_container_corner_x = data->bl_container_corner_x;
            bl_container_corner_y = data->bl_container_corner_y;
            br_container_corner_x = data->br_container_corner_x;
            br_container_corner_y = data->br_container_corner_y;

            tl_cell_guide_crop0_x = data->tl_cell_guide_crop0_x;
            tl_cell_guide_crop0_y = data->tl_cell_guide_crop0_y;
            tr_cell_guide_crop0_x = data->tr_cell_guide_crop0_x;
            tr_cell_guide_crop0_y = data->tr_cell_guide_crop0_y;
            bl_cell_guide_crop0_x = data->bl_cell_guide_crop0_x;
            bl_cell_guide_crop0_y = data->bl_cell_guide_crop0_y;
            br_cell_guide_crop0_x = data->br_cell_guide_crop0_x;
            br_cell_guide_crop0_y = data->br_cell_guide_crop0_y;

            tl_cell_guide_crop1_x = data->tl_cell_guide_crop1_x;
            tl_cell_guide_crop1_y = data->tl_cell_guide_crop1_y;
            tr_cell_guide_crop1_x = data->tr_cell_guide_crop1_x;
            tr_cell_guide_crop1_y = data->tr_cell_guide_crop1_y;
            bl_cell_guide_crop1_x = data->bl_cell_guide_crop1_x;
            bl_cell_guide_crop1_y = data->bl_cell_guide_crop1_y;
            br_cell_guide_crop1_x = data->br_cell_guide_crop1_x;
            br_cell_guide_crop1_y = data->br_cell_guide_crop1_y;

            tl_cell_guide_crop2_x = data->tl_cell_guide_crop2_x;
            tl_cell_guide_crop2_y = data->tl_cell_guide_crop2_y;
            tr_cell_guide_crop2_x = data->tr_cell_guide_crop2_x;
            tr_cell_guide_crop2_y = data->tr_cell_guide_crop2_y;
            bl_cell_guide_crop2_x = data->bl_cell_guide_crop2_x;
            bl_cell_guide_crop2_y = data->bl_cell_guide_crop2_y;
            br_cell_guide_crop2_x = data->br_cell_guide_crop2_x;
            br_cell_guide_crop2_y = data->br_cell_guide_crop2_y;

            tl_container_corner_calibrated.x = xian_tl_container_point_x;
            tl_container_corner_calibrated.y = xian_tl_container_point_y;
            tr_container_corner_calibrated.x = xian_tr_container_point_x;
            tr_container_corner_calibrated.y = xian_tr_container_point_y;
            bl_container_corner_calibrated.x = xian_bl_container_point_x;
            bl_container_corner_calibrated.y = xian_bl_container_point_y;
            br_container_corner_calibrated.x = xian_br_container_point_x;
            br_container_corner_calibrated.y = xian_br_container_point_y;

            tl_container_corner_identified.x = tl_container_corner_x;
            tl_container_corner_identified.y = tl_container_corner_y;
            tr_container_corner_identified.x = tr_container_corner_x;
            tr_container_corner_identified.y = tr_container_corner_y;
            bl_container_corner_identified.x = bl_container_corner_x;
            bl_container_corner_identified.y = bl_container_corner_y;
            br_container_corner_identified.x = br_container_corner_x;
            br_container_corner_identified.y = br_container_corner_y;

            tl_cell_guide_crop0.x = tl_cell_guide_crop0_x;
            tl_cell_guide_crop0.y = tl_cell_guide_crop0_y;
            tr_cell_guide_crop0.x = tr_cell_guide_crop0_x;
            tr_cell_guide_crop0.y = tr_cell_guide_crop0_y;
            bl_cell_guide_crop0.x = bl_cell_guide_crop0_x;
            bl_cell_guide_crop0.y = bl_cell_guide_crop0_y;
            br_cell_guide_crop0.x = br_cell_guide_crop0_x;
            br_cell_guide_crop0.y = br_cell_guide_crop0_y;

            tl_cell_guide_crop1.x = tl_cell_guide_crop1_x;
            tl_cell_guide_crop1.y = tl_cell_guide_crop1_y;
            tr_cell_guide_crop1.x = tr_cell_guide_crop1_x;
            tr_cell_guide_crop1.y = tr_cell_guide_crop1_y;
            bl_cell_guide_crop1.x = bl_cell_guide_crop1_x;
            bl_cell_guide_crop1.y = bl_cell_guide_crop1_y;
            br_cell_guide_crop1.x = br_cell_guide_crop1_x;
            br_cell_guide_crop1.y = br_cell_guide_crop1_y;

            tl_cell_guide_crop2.x = tl_cell_guide_crop2_x;
            tl_cell_guide_crop2.y = tl_cell_guide_crop2_y;
            tr_cell_guide_crop2.x = tr_cell_guide_crop2_x;
            tr_cell_guide_crop2.y = tr_cell_guide_crop2_y;
            bl_cell_guide_crop2.x = bl_cell_guide_crop2_x;
            bl_cell_guide_crop2.y = bl_cell_guide_crop2_y;
            br_cell_guide_crop2.x = br_cell_guide_crop2_x;
            br_cell_guide_crop2.y = br_cell_guide_crop2_y;

            cv::circle(tl_image, tl_container_corner_calibrated, 8, cv::Scalar(0, 0, 255), -1);
            cv::circle(tr_image, tr_container_corner_calibrated, 8, cv::Scalar(0, 0, 255), -1);
            cv::circle(bl_image, bl_container_corner_calibrated, 8, cv::Scalar(0, 0, 255), -1);
            cv::circle(br_image, br_container_corner_calibrated, 8, cv::Scalar(0, 0, 255), -1);
            
            cv::circle(tl_image, tl_container_corner_identified, 8, cv::Scalar(255, 0, 0), -1);
            cv::circle(tr_image, tr_container_corner_identified, 8, cv::Scalar(255, 0, 0), -1);
            cv::circle(bl_image, bl_container_corner_identified, 8, cv::Scalar(255, 0, 0), -1);
            cv::circle(br_image, br_container_corner_identified, 8, cv::Scalar(255, 0, 0), -1);

            cv::circle(tl_image, tl_cell_guide_crop0, 8, cv::Scalar(255, 255, 255), -1);
            cv::circle(tr_image, tr_cell_guide_crop0, 8, cv::Scalar(255, 255, 255), -1);
            cv::circle(bl_image, bl_cell_guide_crop0, 8, cv::Scalar(255, 255, 255), -1);
            cv::circle(br_image, br_cell_guide_crop0, 8, cv::Scalar(255, 255, 255), -1);

            cv::circle(tl_image, tl_cell_guide_crop1, 8, cv::Scalar(0, 255, 255), -1);
            cv::circle(tr_image, tr_cell_guide_crop1, 8, cv::Scalar(0, 255, 255), -1);
            cv::circle(bl_image, bl_cell_guide_crop1, 8, cv::Scalar(0, 255, 255), -1);
            cv::circle(br_image, br_cell_guide_crop1, 8, cv::Scalar(0, 255, 255), -1);

            cv::circle(tl_image, tl_cell_guide_crop2, 8, cv::Scalar(255, 255, 0), -1);
            cv::circle(tr_image, tr_cell_guide_crop2, 8, cv::Scalar(255, 255, 0), -1);
            cv::circle(bl_image, bl_cell_guide_crop2, 8, cv::Scalar(255, 255, 0), -1);
            cv::circle(br_image, br_cell_guide_crop2, 8, cv::Scalar(255, 255, 0), -1);

            cv::Mat src_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            cv::Mat src_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            cv::Mat merge_log = zpmc::zpmc_images_merge_col(src_merge_col_0, src_merge_col_1);
            cv::resize(merge_log, merge_log, cv::Size(merge_log.cols/4, merge_log.rows/4), 2); 

            cv::imshow("xian_keypoints_recognition_show:", merge_log);
            cv::waitKey(1);
            // cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+".jpg", merge_log);

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            std::cout << "FPS: " << 1000.0 / timediff << std::endl;
        } 
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_keypoints_recognition_show_node");
    Xian_KeypointsRecognitionShow xian_keypoints_recognition_show;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_keypoints_recognition_show.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_KeypointsRecognitionShow::m_timer_HeartBeat_f, &xian_keypoints_recognition_show);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}