#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_crop_image_msg.h"
#include "zpmc_cv_control.h"


class Xian_ImageCropProcessShow
{
    public:
        Xian_ImageCropProcessShow()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_crop_image_msg>("xian_crop_images", 1, &Xian_ImageCropProcessShow::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_image_crop_process_show_heart_beat: " << counter << std::endl;
        }

    private:
        ros::Subscriber command_subscribe_;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        std::string timeStr;

        cv::Scalar yellow = cv::Scalar(0, 255, 255);
        cv::Scalar lemon = cv::Scalar(132, 227, 255);
        cv::Scalar blue = cv::Scalar(255, 0, 0);
        cv::Scalar red = cv::Scalar(0, 0, 255);

        int xian_tl_container_point_x = 0;
        int xian_tl_container_point_y = 0;
        int xian_tr_container_point_x = 0;
        int xian_tr_container_point_y = 0;
        int xian_bl_container_point_x = 0;
        int xian_bl_container_point_y = 0;
        int xian_br_container_point_x = 0;
        int xian_br_container_point_y = 0;

        int container_corner_tl_x0 = 0;
        int container_corner_tl_y0 = 0;
        int container_corner_tr_x0 = 0;
        int container_corner_tr_y0 = 0;
        int container_corner_bl_x0 = 0;
        int container_corner_bl_y0 = 0;
        int container_corner_br_x0 = 0;
        int container_corner_br_y0 = 0;

        int clip1_cell_guide_tl_x = 0;
        int clip1_cell_guide_tl_y = 0;
        int clip1_cell_guide_tr_x = 0;
        int clip1_cell_guide_tr_y = 0;
        int clip1_cell_guide_bl_x = 0;
        int clip1_cell_guide_bl_y = 0;
        int clip1_cell_guide_br_x = 0;
        int clip1_cell_guide_br_y = 0;

        int clip2_cell_guide_tl_x = 0;
        int clip2_cell_guide_tl_y = 0;
        int clip2_cell_guide_tr_x = 0;
        int clip2_cell_guide_tr_y = 0;
        int clip2_cell_guide_bl_x = 0;
        int clip2_cell_guide_bl_y = 0;
        int clip2_cell_guide_br_x = 0;
        int clip2_cell_guide_br_y = 0;

        cv::Point tl_container_corner_calibrated = cv::Point(0,0);
        cv::Point tr_container_corner_calibrated = cv::Point(0,0);
        cv::Point bl_container_corner_calibrated = cv::Point(0,0);
        cv::Point br_container_corner_calibrated = cv::Point(0,0);

        cv::Point container_corner_crop_tl = cv::Point(0,0);
        cv::Point container_corner_crop_tr = cv::Point(0,0);
        cv::Point container_corner_crop_bl = cv::Point(0,0);
        cv::Point container_corner_crop_br = cv::Point(0,0);

        cv::Point tl_cell_guide_crop1 = cv::Point(0,0);
        cv::Point tr_cell_guide_crop1 = cv::Point(0,0);
        cv::Point bl_cell_guide_crop1 = cv::Point(0,0);
        cv::Point br_cell_guide_crop1 = cv::Point(0,0);

        cv::Point tl_cell_guide_crop2 = cv::Point(0,0);
        cv::Point tr_cell_guide_crop2 = cv::Point(0,0);
        cv::Point bl_cell_guide_crop2 = cv::Point(0,0);
        cv::Point br_cell_guide_crop2 = cv::Point(0,0);
 
        cv::Mat tl_image, tr_image, bl_image, br_image;

        void command_callback(const xian_msg_pkg::xian_crop_image_msgConstPtr& data)
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

            container_corner_tl_x0 = data->container_corner_tl_x0;
            container_corner_tl_y0 = data->container_corner_tl_y0;
            container_corner_tr_x0 = data->container_corner_tr_x0;
            container_corner_tr_y0 = data->container_corner_tr_y0;
            container_corner_bl_x0 = data->container_corner_bl_x0;
            container_corner_bl_y0 = data->container_corner_bl_y0;
            container_corner_br_x0 = data->container_corner_br_x0;
            container_corner_br_y0 = data->container_corner_br_y0;

            clip1_cell_guide_tl_x = data->clip1_cell_guide_tl_x;
            clip1_cell_guide_tl_y = data->clip1_cell_guide_tl_y;
            clip1_cell_guide_tr_x = data->clip1_cell_guide_tr_x;
            clip1_cell_guide_tr_y = data->clip1_cell_guide_tr_y;
            clip1_cell_guide_bl_x = data->clip1_cell_guide_bl_x;
            clip1_cell_guide_bl_y = data->clip1_cell_guide_bl_y;
            clip1_cell_guide_br_x = data->clip1_cell_guide_br_x;
            clip1_cell_guide_br_y = data->clip1_cell_guide_br_y;

            clip2_cell_guide_tl_x = data->clip2_cell_guide_tl_x;
            clip2_cell_guide_tl_y = data->clip2_cell_guide_tl_y;
            clip2_cell_guide_tr_x = data->clip2_cell_guide_tr_x;
            clip2_cell_guide_tr_y = data->clip2_cell_guide_tr_y;
            clip2_cell_guide_bl_x = data->clip2_cell_guide_bl_x;
            clip2_cell_guide_bl_y = data->clip2_cell_guide_bl_y;
            clip2_cell_guide_br_x = data->clip2_cell_guide_br_x;
            clip2_cell_guide_br_y = data->clip2_cell_guide_br_y;

            tl_container_corner_calibrated.x = xian_tl_container_point_x;
            tl_container_corner_calibrated.y = xian_tl_container_point_y;
            tr_container_corner_calibrated.x = xian_tr_container_point_x;
            tr_container_corner_calibrated.y = xian_tr_container_point_y;
            bl_container_corner_calibrated.x = xian_bl_container_point_x;
            bl_container_corner_calibrated.y = xian_bl_container_point_y;
            br_container_corner_calibrated.x = xian_br_container_point_x;
            br_container_corner_calibrated.y = xian_br_container_point_y;

            container_corner_crop_tl.x = container_corner_tl_x0;
            container_corner_crop_tl.y = container_corner_tl_y0;
            container_corner_crop_tr.x = container_corner_tr_x0;
            container_corner_crop_tr.y = container_corner_tr_y0;
            container_corner_crop_bl.x = container_corner_bl_x0;
            container_corner_crop_bl.y = container_corner_bl_y0;
            container_corner_crop_br.x = container_corner_br_x0;
            container_corner_crop_br.y = container_corner_br_y0;

            tl_cell_guide_crop1.x = clip1_cell_guide_tl_x;
            tl_cell_guide_crop1.y = clip1_cell_guide_tl_y;
            tr_cell_guide_crop1.x = clip1_cell_guide_tr_x;
            tr_cell_guide_crop1.y = clip1_cell_guide_tr_y;
            bl_cell_guide_crop1.x = clip1_cell_guide_bl_x;
            bl_cell_guide_crop1.y = clip1_cell_guide_bl_y;
            br_cell_guide_crop1.x = clip1_cell_guide_br_x;
            br_cell_guide_crop1.y = clip1_cell_guide_br_y;

            tl_cell_guide_crop2.x = clip2_cell_guide_tl_x;
            tl_cell_guide_crop2.y = clip2_cell_guide_tl_y;
            tr_cell_guide_crop2.x = clip2_cell_guide_tr_x;
            tr_cell_guide_crop2.y = clip2_cell_guide_tr_y;
            bl_cell_guide_crop2.x = clip2_cell_guide_bl_x;
            bl_cell_guide_crop2.y = clip2_cell_guide_bl_y;
            br_cell_guide_crop2.x = clip2_cell_guide_br_x;
            br_cell_guide_crop2.y = clip2_cell_guide_br_y;

            cv::circle(tl_image, tl_container_corner_calibrated, 16, red, -1);
            cv::circle(tr_image, tr_container_corner_calibrated, 16, red, -1);
            cv::circle(bl_image, bl_container_corner_calibrated, 16, red, -1);
            cv::circle(br_image, br_container_corner_calibrated, 16, red, -1);

            int tl_cell_guide_crop1_x = tl_cell_guide_crop1.x + 256;
            int tl_cell_guide_crop1_y = tl_cell_guide_crop1.y + 256;
            int tr_cell_guide_crop1_x = tr_cell_guide_crop1.x + 256;
            int tr_cell_guide_crop1_y = tr_cell_guide_crop1.y + 256;
            int bl_cell_guide_crop1_x = bl_cell_guide_crop1.x + 256;
            int bl_cell_guide_crop1_y = bl_cell_guide_crop1.y + 256;
            int br_cell_guide_crop1_x = br_cell_guide_crop1.x + 256;
            int br_cell_guide_crop1_y = br_cell_guide_crop1.y + 256;
            cv::Point tl_cell_guide_crop1_xy1 = cv::Point(tl_cell_guide_crop1_x, tl_cell_guide_crop1_y);
            cv::Point tr_cell_guide_crop1_xy1 = cv::Point(tr_cell_guide_crop1_x, tr_cell_guide_crop1_y);
            cv::Point bl_cell_guide_crop1_xy1 = cv::Point(bl_cell_guide_crop1_x, bl_cell_guide_crop1_y);
            cv::Point br_cell_guide_crop1_xy1 = cv::Point(br_cell_guide_crop1_x, br_cell_guide_crop1_y);
            cv::rectangle(tl_image, tl_cell_guide_crop1, tl_cell_guide_crop1_xy1, yellow, 4); 
            cv::rectangle(tr_image, tr_cell_guide_crop1, tr_cell_guide_crop1_xy1, yellow, 4); 
            cv::rectangle(bl_image, bl_cell_guide_crop1, bl_cell_guide_crop1_xy1, yellow, 4); 
            cv::rectangle(br_image, br_cell_guide_crop1, br_cell_guide_crop1_xy1, yellow, 4); 

            int tl_cell_guide_crop2_x = tl_cell_guide_crop2.x + 256;
            int tl_cell_guide_crop2_y = tl_cell_guide_crop2.y + 256;
            int tr_cell_guide_crop2_x = tr_cell_guide_crop2.x + 256;
            int tr_cell_guide_crop2_y = tr_cell_guide_crop2.y + 256;
            int bl_cell_guide_crop2_x = bl_cell_guide_crop2.x + 256;
            int bl_cell_guide_crop2_y = bl_cell_guide_crop2.y + 256;
            int br_cell_guide_crop2_x = br_cell_guide_crop2.x + 256;
            int br_cell_guide_crop2_y = br_cell_guide_crop2.y + 256;
            cv::Point tl_cell_guide_crop1_xy2 = cv::Point(tl_cell_guide_crop2_x, tl_cell_guide_crop2_y);
            cv::Point tr_cell_guide_crop1_xy2 = cv::Point(tr_cell_guide_crop2_x, tr_cell_guide_crop2_y);
            cv::Point bl_cell_guide_crop1_xy2 = cv::Point(bl_cell_guide_crop2_x, bl_cell_guide_crop2_y);
            cv::Point br_cell_guide_crop1_xy2 = cv::Point(br_cell_guide_crop2_x, br_cell_guide_crop2_y);
            cv::rectangle(tl_image, tl_cell_guide_crop2, tl_cell_guide_crop1_xy2, blue, 4); 
            cv::rectangle(tr_image, tr_cell_guide_crop2, tr_cell_guide_crop1_xy2, blue, 4); 
            cv::rectangle(bl_image, bl_cell_guide_crop2, bl_cell_guide_crop1_xy2, blue, 4); 
            cv::rectangle(br_image, br_cell_guide_crop2, br_cell_guide_crop1_xy2, blue, 4); 

            cv::Mat src_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            cv::Mat src_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            cv::Mat merge_log = zpmc::zpmc_images_merge_col(src_merge_col_0, src_merge_col_1);
            // cv::resize(merge_log, merge_log, cv::Size(merge_log.cols/4, merge_log.rows/4), 2); 

            // cv::imshow("images:", merge_log);
            // cv::waitKey(1);
            cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+".jpg", merge_log);

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            std::cout << "FPS: " << 1000.0 / timediff << std::endl;
        } 
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_image_crop_process_show_node");
    Xian_ImageCropProcessShow xian_image_crop_process_show;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_image_crop_process_show.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_ImageCropProcessShow::m_timer_HeartBeat_f, &xian_image_crop_process_show);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}