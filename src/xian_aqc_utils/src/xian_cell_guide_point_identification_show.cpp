#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_cell_guide_point_identification.h"
#include "zpmc_cv_control.h"


class Xian_CellGuidePointIdentificationShow
{
    public:
        Xian_CellGuidePointIdentificationShow()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_cell_guide_point_identification>("xian_cell_guide_points", 1, &Xian_CellGuidePointIdentificationShow::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_cell_guide_point_identification_show_heart_beat: " << counter << std::endl;
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

        int tl_container_corner_cx = 0;
        int tl_container_corner_cy = 0;
        int tr_container_corner_cx = 0;
        int tr_container_corner_cy = 0;
        int bl_container_corner_cx = 0;
        int bl_container_corner_cy = 0;
        int br_container_corner_cx = 0;
        int br_container_corner_cy = 0;

        int tl_cell_guide0_cx = 0;
        int tl_cell_guide0_cy = 0;
        int tr_cell_guide0_cx = 0;
        int tr_cell_guide0_cy = 0;
        int bl_cell_guide0_cx = 0;
        int bl_cell_guide0_cy = 0;
        int br_cell_guide0_cx = 0;
        int br_cell_guide0_cy = 0;

        int tl_cell_guide1_cx = 0;
        int tl_cell_guide1_cy = 0;
        int tr_cell_guide1_cx = 0;
        int tr_cell_guide1_cy = 0;
        int bl_cell_guide1_cx = 0;
        int bl_cell_guide1_cy = 0;
        int br_cell_guide1_cx = 0;
        int br_cell_guide1_cy = 0;

        int tl_cell_guide2_cx = 0;
        int tl_cell_guide2_cy = 0;
        int tr_cell_guide2_cx = 0;
        int tr_cell_guide2_cy = 0;
        int bl_cell_guide2_cx = 0;
        int bl_cell_guide2_cy = 0;
        int br_cell_guide2_cx = 0;
        int br_cell_guide2_cy = 0;

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

        cv::Point tl_container_corner = cv::Point(0,0);
        cv::Point tr_container_corner = cv::Point(0,0);
        cv::Point bl_container_corner = cv::Point(0,0);
        cv::Point br_container_corner = cv::Point(0,0);

        cv::Point tl_cell_guide0_c = cv::Point(0,0);
        cv::Point tr_cell_guide0_c = cv::Point(0,0);
        cv::Point bl_cell_guide0_c = cv::Point(0,0);
        cv::Point br_cell_guide0_c = cv::Point(0,0);
        cv::Point tl_cell_guide1_c = cv::Point(0,0);
        cv::Point tr_cell_guide1_c = cv::Point(0,0);
        cv::Point bl_cell_guide1_c = cv::Point(0,0);
        cv::Point br_cell_guide1_c = cv::Point(0,0);
        cv::Point tl_cell_guide2_c = cv::Point(0,0);
        cv::Point tr_cell_guide2_c = cv::Point(0,0);
        cv::Point bl_cell_guide2_c = cv::Point(0,0);
        cv::Point br_cell_guide2_c = cv::Point(0,0);

        cv::Point tl_cell_guide0_c_target = cv::Point(0,0);
        cv::Point tr_cell_guide0_c_target = cv::Point(0,0);
        cv::Point bl_cell_guide0_c_target = cv::Point(0,0);
        cv::Point br_cell_guide0_c_target = cv::Point(0,0);
        cv::Point tl_cell_guide1_c_target = cv::Point(0,0);
        cv::Point tr_cell_guide1_c_target = cv::Point(0,0);
        cv::Point bl_cell_guide1_c_target = cv::Point(0,0);
        cv::Point br_cell_guide1_c_target = cv::Point(0,0);
        cv::Point tl_cell_guide2_c_target = cv::Point(0,0);
        cv::Point tr_cell_guide2_c_target = cv::Point(0,0);
        cv::Point bl_cell_guide2_c_target = cv::Point(0,0);
        cv::Point br_cell_guide2_c_target = cv::Point(0,0);
 
        cv::Mat tl_image, tr_image, bl_image, br_image;
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

        void command_callback(const xian_msg_pkg::xian_cell_guide_point_identificationConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            std::cout << "Time:" << timeStr << std::endl;

            tl_image = cv_bridge::toCvShare(data->tl_image, data, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(data->tr_image, data, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(data->bl_image, data, "bgr8")->image;
            br_image = cv_bridge::toCvShare(data->br_image, data, "bgr8")->image;

            tl_mask_resize_0_show = cv_bridge::toCvShare(data->tl_mask_resize_0_show, data, "bgr8")->image; 
            tr_mask_resize_0_show = cv_bridge::toCvShare(data->tr_mask_resize_0_show, data, "bgr8")->image; 
            bl_mask_resize_0_show = cv_bridge::toCvShare(data->bl_mask_resize_0_show, data, "bgr8")->image;
            br_mask_resize_0_show = cv_bridge::toCvShare(data->br_mask_resize_0_show, data, "bgr8")->image;
            tl_mask_resize_1_show = cv_bridge::toCvShare(data->tl_mask_resize_1_show, data, "bgr8")->image; 
            tr_mask_resize_1_show = cv_bridge::toCvShare(data->tr_mask_resize_1_show, data, "bgr8")->image; 
            bl_mask_resize_1_show = cv_bridge::toCvShare(data->bl_mask_resize_1_show, data, "bgr8")->image;
            br_mask_resize_1_show = cv_bridge::toCvShare(data->br_mask_resize_1_show, data, "bgr8")->image;
            tl_mask_resize_2_show = cv_bridge::toCvShare(data->tl_mask_resize_2_show, data, "bgr8")->image; 
            tr_mask_resize_2_show = cv_bridge::toCvShare(data->tr_mask_resize_2_show, data, "bgr8")->image; 
            bl_mask_resize_2_show = cv_bridge::toCvShare(data->bl_mask_resize_2_show, data, "bgr8")->image;
            br_mask_resize_2_show = cv_bridge::toCvShare(data->br_mask_resize_2_show, data, "bgr8")->image;
            
            tl_container_corner_cx = data->tl_container_corner_cx;
            tl_container_corner_cy = data->tl_container_corner_cy;
            tr_container_corner_cx = data->tr_container_corner_cx;
            tr_container_corner_cy = data->tr_container_corner_cy;
            bl_container_corner_cx = data->bl_container_corner_cx;
            bl_container_corner_cy = data->bl_container_corner_cy;
            br_container_corner_cx = data->br_container_corner_cx;
            br_container_corner_cy = data->br_container_corner_cy;

            tl_container_corner.x = tl_container_corner_cx;
            tl_container_corner.y = tl_container_corner_cy;
            tr_container_corner.x = tr_container_corner_cx;
            tr_container_corner.y = tr_container_corner_cy;
            bl_container_corner.x = bl_container_corner_cx;
            bl_container_corner.y = bl_container_corner_cy;
            br_container_corner.x = br_container_corner_cx;
            br_container_corner.y = br_container_corner_cy;

            tl_cell_guide0_cx = data->tl_cell_guide0_cx;
            tl_cell_guide0_cy = data->tl_cell_guide0_cy;
            tr_cell_guide0_cx = data->tr_cell_guide0_cx;
            tr_cell_guide0_cy = data->tr_cell_guide0_cy;
            bl_cell_guide0_cx = data->bl_cell_guide0_cx;
            bl_cell_guide0_cy = data->bl_cell_guide0_cy;
            br_cell_guide0_cx = data->br_cell_guide0_cx;
            br_cell_guide0_cy = data->br_cell_guide0_cy;
            tl_cell_guide1_cx = data->tl_cell_guide1_cx;
            tl_cell_guide1_cy = data->tl_cell_guide1_cy;
            tr_cell_guide1_cx = data->tr_cell_guide1_cx;
            tr_cell_guide1_cy = data->tr_cell_guide1_cy;
            bl_cell_guide1_cx = data->bl_cell_guide1_cx;
            bl_cell_guide1_cy = data->bl_cell_guide1_cy;
            br_cell_guide1_cx = data->br_cell_guide1_cx;
            br_cell_guide1_cy = data->br_cell_guide1_cy;
            tl_cell_guide2_cx = data->tl_cell_guide2_cx;
            tl_cell_guide2_cy = data->tl_cell_guide2_cy;
            tr_cell_guide2_cx = data->tr_cell_guide2_cx;
            tr_cell_guide2_cy = data->tr_cell_guide2_cy;
            bl_cell_guide2_cx = data->bl_cell_guide2_cx;
            bl_cell_guide2_cy = data->bl_cell_guide2_cy;
            br_cell_guide2_cx = data->br_cell_guide2_cx;
            br_cell_guide2_cy = data->br_cell_guide2_cy;

            tl_cell_guide0_c.x = tl_cell_guide0_cx;
            tl_cell_guide0_c.y = tl_cell_guide0_cy;
            tr_cell_guide0_c.x = tr_cell_guide0_cx;
            tr_cell_guide0_c.y = tr_cell_guide0_cy;
            bl_cell_guide0_c.x = bl_cell_guide0_cx;
            bl_cell_guide0_c.y = bl_cell_guide0_cy;
            br_cell_guide0_c.x = br_cell_guide0_cx;
            br_cell_guide0_c.y = br_cell_guide0_cy;
            tl_cell_guide1_c.x = tl_cell_guide1_cx;
            tl_cell_guide1_c.y = tl_cell_guide1_cy;
            tr_cell_guide1_c.x = tr_cell_guide1_cx;
            tr_cell_guide1_c.y = tr_cell_guide1_cy;
            bl_cell_guide1_c.x = bl_cell_guide1_cx;
            bl_cell_guide1_c.y = bl_cell_guide1_cy;
            br_cell_guide1_c.x = br_cell_guide1_cx;
            br_cell_guide1_c.y = br_cell_guide1_cy;
            tl_cell_guide2_c.x = tl_cell_guide2_cx;
            tl_cell_guide2_c.y = tl_cell_guide2_cy;
            tr_cell_guide2_c.x = tr_cell_guide2_cx;
            tr_cell_guide2_c.y = tr_cell_guide2_cy;
            bl_cell_guide2_c.x = bl_cell_guide2_cx;
            bl_cell_guide2_c.y = bl_cell_guide2_cy;
            br_cell_guide2_c.x = br_cell_guide2_cx;
            br_cell_guide2_c.y = br_cell_guide2_cy;

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

            tl_cell_guide0_c_target.x = tl_cell_guide0_c.x+tl_cell_guide_crop0_tl_x;
            tl_cell_guide0_c_target.y = tl_cell_guide0_c.y+tl_cell_guide_crop0_tl_y;
            tr_cell_guide0_c_target.x = tr_cell_guide0_c.x+tr_cell_guide_crop0_tl_x;
            tr_cell_guide0_c_target.y = tr_cell_guide0_c.y+tr_cell_guide_crop0_tl_y;
            bl_cell_guide0_c_target.x = bl_cell_guide0_c.x+bl_cell_guide_crop0_tl_x;
            bl_cell_guide0_c_target.y = bl_cell_guide0_c.y+bl_cell_guide_crop0_tl_y;
            br_cell_guide0_c_target.x = br_cell_guide0_c.x+br_cell_guide_crop0_tl_x;
            br_cell_guide0_c_target.y = br_cell_guide0_c.y+br_cell_guide_crop0_tl_y;
            tl_cell_guide1_c_target.x = tl_cell_guide1_c.x+tl_cell_guide_crop1_tl_x;
            tl_cell_guide1_c_target.y = tl_cell_guide1_c.y+tl_cell_guide_crop1_tl_y;
            tr_cell_guide1_c_target.x = tr_cell_guide1_c.x+tr_cell_guide_crop1_tl_x;
            tr_cell_guide1_c_target.y = tr_cell_guide1_c.y+tr_cell_guide_crop1_tl_y;
            bl_cell_guide1_c_target.x = bl_cell_guide1_c.x+bl_cell_guide_crop1_tl_x;
            bl_cell_guide1_c_target.y = bl_cell_guide1_c.y+bl_cell_guide_crop1_tl_y;
            br_cell_guide1_c_target.x = br_cell_guide1_c.x+br_cell_guide_crop1_tl_x;
            br_cell_guide1_c_target.y = br_cell_guide1_c.y+br_cell_guide_crop1_tl_y;
            tl_cell_guide2_c_target.x = tl_cell_guide2_c.x+tl_cell_guide_crop2_tl_x;
            tl_cell_guide2_c_target.y = tl_cell_guide2_c.y+tl_cell_guide_crop2_tl_y;
            tr_cell_guide2_c_target.x = tr_cell_guide2_c.x+tr_cell_guide_crop2_tl_x;
            tr_cell_guide2_c_target.y = tr_cell_guide2_c.y+tr_cell_guide_crop2_tl_y;
            bl_cell_guide2_c_target.x = bl_cell_guide2_c.x+bl_cell_guide_crop2_tl_x;
            bl_cell_guide2_c_target.y = bl_cell_guide2_c.y+bl_cell_guide_crop2_tl_y;
            br_cell_guide2_c_target.x = br_cell_guide2_c.x+br_cell_guide_crop2_tl_x;
            br_cell_guide2_c_target.y = br_cell_guide2_c.y+br_cell_guide_crop2_tl_y;

            cv::circle(tl_image, tl_container_corner, 16, red, -1);
            cv::circle(tr_image, tr_container_corner, 16, red, -1);
            cv::circle(bl_image, bl_container_corner, 16, red, -1);
            cv::circle(br_image, br_container_corner, 16, red, -1);

            tl_image = draw_cell_guide_point(tl_cell_guide0_c, tl_cell_guide1_c, tl_cell_guide2_c, 
                                             tl_cell_guide0_c_target, tl_cell_guide1_c_target, tl_cell_guide2_c_target, 
                                             tl_image, yellow);
            tr_image = draw_cell_guide_point(tr_cell_guide0_c, tr_cell_guide1_c, tr_cell_guide2_c, 
                                             tr_cell_guide0_c_target, tr_cell_guide1_c_target, tr_cell_guide2_c_target, 
                                             tr_image, yellow);
            bl_image = draw_cell_guide_point(bl_cell_guide0_c, bl_cell_guide1_c, bl_cell_guide2_c, 
                                             bl_cell_guide0_c_target, bl_cell_guide1_c_target, bl_cell_guide2_c_target, 
                                             bl_image, yellow);
            br_image = draw_cell_guide_point(br_cell_guide0_c, br_cell_guide1_c, br_cell_guide2_c, 
                                             br_cell_guide0_c_target, br_cell_guide1_c_target, br_cell_guide2_c_target, 
                                             br_image, yellow);

            cv::Mat tl_mask = cv::Mat::zeros(tl_image.size(),tl_image.type());
            cv::Mat tr_mask = cv::Mat::zeros(tr_image.size(),tr_image.type());
            cv::Mat bl_mask = cv::Mat::zeros(bl_image.size(),tl_image.type());
            cv::Mat br_mask = cv::Mat::zeros(br_image.size(),tr_image.type());


            cv::Mat* masks0 = copy_to_image(tl_mask_resize_0_show, tr_mask_resize_0_show, 
                                            bl_mask_resize_0_show, br_mask_resize_0_show, 
                                            tl_cell_guide_crop0_tl_x, tl_cell_guide_crop0_tl_y,
                                            tr_cell_guide_crop0_tl_x, tr_cell_guide_crop0_tl_y,
                                            bl_cell_guide_crop0_tl_x, bl_cell_guide_crop0_tl_y,
                                            br_cell_guide_crop0_tl_x, br_cell_guide_crop0_tl_y,
                                            tl_mask, tr_mask, bl_mask, br_mask);
            tl_mask = *(masks0+0);
            tr_mask = *(masks0+1);
            bl_mask = *(masks0+2);
            br_mask = *(masks0+3);

            cv::Mat* masks1 = copy_to_image(tl_mask_resize_1_show, tr_mask_resize_1_show, 
                                            bl_mask_resize_1_show, br_mask_resize_1_show, 
                                            tl_cell_guide_crop1_tl_x, tl_cell_guide_crop1_tl_y,
                                            tr_cell_guide_crop1_tl_x, tr_cell_guide_crop1_tl_y,
                                            bl_cell_guide_crop1_tl_x, bl_cell_guide_crop1_tl_y,
                                            br_cell_guide_crop1_tl_x, br_cell_guide_crop1_tl_y,
                                            tl_mask, tr_mask, bl_mask, br_mask);
            tl_mask = *(masks1+0);
            tr_mask = *(masks1+1);
            bl_mask = *(masks1+2);
            br_mask = *(masks1+3);

            cv::Mat* masks2 = copy_to_image(tl_mask_resize_2_show, tr_mask_resize_2_show, 
                                            bl_mask_resize_2_show, br_mask_resize_2_show, 
                                            tl_cell_guide_crop2_tl_x, tl_cell_guide_crop2_tl_y,
                                            tr_cell_guide_crop2_tl_x, tr_cell_guide_crop2_tl_y,
                                            bl_cell_guide_crop2_tl_x, bl_cell_guide_crop2_tl_y,
                                            br_cell_guide_crop2_tl_x, br_cell_guide_crop2_tl_y,
                                            tl_mask, tr_mask, bl_mask, br_mask);
            tl_mask = *(masks2+0);
            tr_mask = *(masks2+1);
            bl_mask = *(masks2+2);
            br_mask = *(masks2+3);

            
            cv::Mat mask_merge_col_0 = zpmc::zpmc_images_merge_row(tl_mask, bl_mask);
            cv::Mat mask_merge_col_1 = zpmc::zpmc_images_merge_row(tr_mask, br_mask);
            
            cv::Mat src_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            cv::Mat src_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            cv::Mat merge_log = zpmc::zpmc_images_merge_col(src_merge_col_0, src_merge_col_1);

            cv::Mat merge_row0 = zpmc::zpmc_images_merge_col(mask_merge_col_0, merge_log);
            cv::Mat merge_row1 = zpmc::zpmc_images_merge_col(merge_row0, mask_merge_col_1);


            cv::resize(merge_row1, merge_row1, cv::Size(merge_log.cols/2, merge_log.rows/4), 2); 

            cv::imshow("xian_cell_guide_point_identification_show:", merge_row1);
            cv::waitKey(1);
            // cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+".jpg", merge_row1);
            
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            std::cout << "FPS: " << 1000.0 / timediff << std::endl;
        } 

        cv::Mat draw_cell_guide_point(cv::Point xy0, cv::Point xy1, cv::Point xy2, 
                                      cv::Point xy0_target, cv::Point xy1_target, cv::Point xy2_target,
                                      cv::Mat img, cv::Scalar color)
        {
            int counter = 0;

            printf("xy0: %d, %d \n", xy0.x, xy0.y);
            printf("xy1: %d, %d \n", xy1.x, xy1.y);
            printf("xy2: %d, %d \n", xy2.x, xy2.y);

            if(xy0.x == -1 && xy0.y == -1)
            {
                counter+=1;
            }
            else
            {
                cv::circle(img, xy0_target, 16, color, -1);
            }
            
            if(xy1.x == -1 && xy1.y == -1)
            {
                counter+=1;
            }
            else
            {
                cv::circle(img, xy1_target, 16, color, -1);
            }

            if(xy2.x == -1 && xy2.y == -1)
            {
                counter+=1;
            }
            else
            {
                cv::circle(img, xy2_target, 16, color, -1);
            }

            if(counter == 3)
            {
                cv::putText(img,"Can't get cell guide roi!", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, color, 5, 8);
            }
            return img;
        }

        cv::Mat* copy_to_image(cv::Mat tl_crop_image, cv::Mat tr_crop_image, cv::Mat bl_crop_image, cv::Mat br_crop_image, 
                               int tl_x0, int tl_y0, int tr_x0, int tr_y0, int bl_x0, int bl_y0, int br_x0, int br_y0,
                               cv::Mat tl_mask, cv::Mat tr_mask, cv::Mat bl_mask, cv::Mat br_mask)
        {
            static cv::Mat imgs[4];
            

            if(tl_x0 == 0 && tl_y0 == 0)
            {

            }
            else
            {
                cv::resize(tl_crop_image, tl_crop_image, cv::Size(256, 256), 2);
                tl_mask = zpmc::crop_copy_to_mask(tl_x0, tl_y0, tl_mask, tl_crop_image);
            }
            
            if(tr_x0 == 0 && tr_y0 == 0)
            {

            }
            else
            {
                cv::resize(tr_crop_image, tr_crop_image, cv::Size(256, 256), 2);
                tr_mask = zpmc::crop_copy_to_mask(tr_x0, tr_y0, tr_mask, tr_crop_image);
            }
            
            if(bl_x0 == 0 && bl_y0 == 0)
            {

            }
            else
            {
                cv::resize(bl_crop_image, bl_crop_image, cv::Size(256, 256), 2);
                bl_mask = zpmc::crop_copy_to_mask(bl_x0, bl_y0, bl_mask, bl_crop_image);
            }
            
            if(br_x0 == 0 && br_y0 == 0)
            {

            }
            else
            {
                cv::resize(br_crop_image, br_crop_image, cv::Size(256, 256), 2);
                br_mask = zpmc::crop_copy_to_mask(br_x0, br_y0, br_mask, br_crop_image);
            }
            imgs[0] = tl_mask;
            imgs[1] = tr_mask;
            imgs[2] = bl_mask;
            imgs[3] = br_mask;
            return imgs;
        }

        
    
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_cell_guide_point_identification_show_node");
    Xian_CellGuidePointIdentificationShow xian_cell_guide_point_identification_show;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_cell_guide_point_identification_show.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_CellGuidePointIdentificationShow::m_timer_HeartBeat_f, &xian_cell_guide_point_identification_show);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}