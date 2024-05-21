#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_cel_guide_roi_msg.h"
#include "zpmc_cv_control.h"


class Xian_GetCellGuideRoiShow
{
    public:
        Xian_GetCellGuideRoiShow()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_cel_guide_roi_msg>("xian_cell_guide_crop_images", 1, &Xian_GetCellGuideRoiShow::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_get_cell_guide_roi_show_heart_beat: " << counter << std::endl;
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

        cv::Point tl_cell_guide_crop0_tl = cv::Point(0,0);
        cv::Point tr_cell_guide_crop0_tl = cv::Point(0,0);
        cv::Point bl_cell_guide_crop0_tl = cv::Point(0,0);
        cv::Point br_cell_guide_crop0_tl = cv::Point(0,0);

        cv::Point tl_cell_guide_crop1_tl = cv::Point(0,0);
        cv::Point tr_cell_guide_crop1_tl = cv::Point(0,0);
        cv::Point bl_cell_guide_crop1_tl = cv::Point(0,0);
        cv::Point br_cell_guide_crop1_tl = cv::Point(0,0);

        cv::Point tl_cell_guide_crop2_tl = cv::Point(0,0);
        cv::Point tr_cell_guide_crop2_tl = cv::Point(0,0);
        cv::Point bl_cell_guide_crop2_tl = cv::Point(0,0);
        cv::Point br_cell_guide_crop2_tl = cv::Point(0,0);
 
        cv::Mat tl_image, tr_image, bl_image, br_image;

        void command_callback(const xian_msg_pkg::xian_cel_guide_roi_msgConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();

            tl_image = cv_bridge::toCvShare(data->tl_image, data, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(data->tr_image, data, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(data->bl_image, data, "bgr8")->image;
            br_image = cv_bridge::toCvShare(data->br_image, data, "bgr8")->image;
            
            tl_container_corner_cx = data->tl_container_corner_cx;
            tl_container_corner_cy = data->tl_container_corner_cy;
            tr_container_corner_cx = data->tr_container_corner_cx;
            tr_container_corner_cy = data->tr_container_corner_cy;
            bl_container_corner_cx = data->bl_container_corner_cx;
            bl_container_corner_cy = data->bl_container_corner_cy;
            br_container_corner_cx = data->br_container_corner_cx;
            br_container_corner_cy = data->br_container_corner_cy;

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

            tl_container_corner.x = tl_container_corner_cx;
            tl_container_corner.y = tl_container_corner_cy;
            tr_container_corner.x = tr_container_corner_cx;
            tr_container_corner.y = tr_container_corner_cy;
            bl_container_corner.x = bl_container_corner_cx;
            bl_container_corner.y = bl_container_corner_cy;
            br_container_corner.x = br_container_corner_cx;
            br_container_corner.y = br_container_corner_cy;

            tl_cell_guide_crop0_tl.x = tl_cell_guide_crop0_tl_x;
            tl_cell_guide_crop0_tl.y = tl_cell_guide_crop0_tl_y;
            tr_cell_guide_crop0_tl.x = tr_cell_guide_crop0_tl_x;
            tr_cell_guide_crop0_tl.y = tr_cell_guide_crop0_tl_y;
            bl_cell_guide_crop0_tl.x = bl_cell_guide_crop0_tl_x;
            bl_cell_guide_crop0_tl.y = bl_cell_guide_crop0_tl_y;
            br_cell_guide_crop0_tl.x = br_cell_guide_crop0_tl_x;
            br_cell_guide_crop0_tl.y = br_cell_guide_crop0_tl_y;

            tl_cell_guide_crop1_tl.x = tl_cell_guide_crop1_tl_x;
            tl_cell_guide_crop1_tl.y = tl_cell_guide_crop1_tl_y;
            tr_cell_guide_crop1_tl.x = tr_cell_guide_crop1_tl_x;
            tr_cell_guide_crop1_tl.y = tr_cell_guide_crop1_tl_y;
            bl_cell_guide_crop1_tl.x = bl_cell_guide_crop1_tl_x;
            bl_cell_guide_crop1_tl.y = bl_cell_guide_crop1_tl_y;
            br_cell_guide_crop1_tl.x = br_cell_guide_crop1_tl_x;
            br_cell_guide_crop1_tl.y = br_cell_guide_crop1_tl_y;

            tl_cell_guide_crop2_tl.x = tl_cell_guide_crop2_tl_x;
            tl_cell_guide_crop2_tl.y = tl_cell_guide_crop2_tl_y;
            tr_cell_guide_crop2_tl.x = tr_cell_guide_crop2_tl_x;
            tr_cell_guide_crop2_tl.y = tr_cell_guide_crop2_tl_y;
            bl_cell_guide_crop2_tl.x = bl_cell_guide_crop2_tl_x;
            bl_cell_guide_crop2_tl.y = bl_cell_guide_crop2_tl_y;
            br_cell_guide_crop2_tl.x = br_cell_guide_crop2_tl_x;
            br_cell_guide_crop2_tl.y = br_cell_guide_crop2_tl_y;

            cv::circle(tl_image, tl_container_corner, 16, red, -1);
            cv::circle(tr_image, tr_container_corner, 16, red, -1);
            cv::circle(bl_image, bl_container_corner, 16, red, -1);
            cv::circle(br_image, br_container_corner, 16, red, -1);

            cv::Mat* cell_guide_list0 = draw_rectangle(tl_cell_guide_crop0_tl, tr_cell_guide_crop0_tl, bl_cell_guide_crop0_tl, br_cell_guide_crop0_tl,
                                                            tl_image, tr_image, bl_image, br_image, lemon);
            tl_image = *(cell_guide_list0+0);
            tr_image = *(cell_guide_list0+1);
            bl_image = *(cell_guide_list0+2);
            br_image = *(cell_guide_list0+3);

            cv::Mat* cell_guide_list1 = draw_rectangle(tl_cell_guide_crop1_tl, tr_cell_guide_crop1_tl, bl_cell_guide_crop1_tl, br_cell_guide_crop1_tl,
                                                       tl_image, tr_image, bl_image, br_image, yellow);
            tl_image = *(cell_guide_list1+0);
            tr_image = *(cell_guide_list1+1);
            bl_image = *(cell_guide_list1+2);
            br_image = *(cell_guide_list1+3);

            cv::Mat* cell_guide_list2 = draw_rectangle(tl_cell_guide_crop2_tl, tr_cell_guide_crop2_tl, bl_cell_guide_crop2_tl, br_cell_guide_crop2_tl,
                                                       tl_image, tr_image, bl_image, br_image, blue);
            tl_image = *(cell_guide_list2+0);
            tr_image = *(cell_guide_list2+1);
            bl_image = *(cell_guide_list2+2);
            br_image = *(cell_guide_list2+3);


            cv::Mat tl_cell_guide_crop0 = cv_bridge::toCvShare(data->tl_cell_guide_crop0, data, "bgr8")->image; 
            cv::Mat tr_cell_guide_crop0 = cv_bridge::toCvShare(data->tr_cell_guide_crop0, data, "bgr8")->image; 
            cv::Mat bl_cell_guide_crop0 = cv_bridge::toCvShare(data->bl_cell_guide_crop0, data, "bgr8")->image;
            cv::Mat br_cell_guide_crop0 = cv_bridge::toCvShare(data->br_cell_guide_crop0, data, "bgr8")->image;

            cv::Mat tl_cell_guide_crop1 = cv_bridge::toCvShare(data->tl_cell_guide_crop1, data, "bgr8")->image; 
            cv::Mat tr_cell_guide_crop1 = cv_bridge::toCvShare(data->tr_cell_guide_crop1, data, "bgr8")->image; 
            cv::Mat bl_cell_guide_crop1 = cv_bridge::toCvShare(data->bl_cell_guide_crop1, data, "bgr8")->image;
            cv::Mat br_cell_guide_crop1 = cv_bridge::toCvShare(data->br_cell_guide_crop1, data, "bgr8")->image;

            cv::Mat tl_cell_guide_crop2 = cv_bridge::toCvShare(data->tl_cell_guide_crop2, data, "bgr8")->image; 
            cv::Mat tr_cell_guide_crop2 = cv_bridge::toCvShare(data->tr_cell_guide_crop2, data, "bgr8")->image; 
            cv::Mat bl_cell_guide_crop2 = cv_bridge::toCvShare(data->bl_cell_guide_crop2, data, "bgr8")->image;
            cv::Mat br_cell_guide_crop2 = cv_bridge::toCvShare(data->br_cell_guide_crop2, data, "bgr8")->image;

            cv::Mat tl_mask = cv::Mat::zeros(tl_image.size(),tl_image.type());
            cv::Mat tr_mask = cv::Mat::zeros(tr_image.size(),tr_image.type());
            cv::Mat bl_mask = cv::Mat::zeros(bl_image.size(),tl_image.type());
            cv::Mat br_mask = cv::Mat::zeros(br_image.size(),tr_image.type());

            cv::Mat* masks0 = copy_to_image(tl_cell_guide_crop0, tr_cell_guide_crop0, bl_cell_guide_crop0, br_cell_guide_crop0, 
                                           tl_cell_guide_crop0_tl.x, tl_cell_guide_crop0_tl.y, 
                                           tr_cell_guide_crop0_tl.x, tr_cell_guide_crop0_tl.y,
                                           bl_cell_guide_crop0_tl.x, bl_cell_guide_crop0_tl.y, 
                                           br_cell_guide_crop0_tl.x, br_cell_guide_crop0_tl.y,
                                           tl_mask, tr_mask, bl_mask, br_mask);
            tl_mask = *(masks0+0);
            tr_mask = *(masks0+1);
            bl_mask = *(masks0+2);
            br_mask = *(masks0+3);

            cv::Mat* masks1 = copy_to_image(tl_cell_guide_crop1, tr_cell_guide_crop1, bl_cell_guide_crop1, br_cell_guide_crop1, 
                                           tl_cell_guide_crop1_tl.x, tl_cell_guide_crop1_tl.y, 
                                           tr_cell_guide_crop1_tl.x, tr_cell_guide_crop1_tl.y,
                                           bl_cell_guide_crop1_tl.x, bl_cell_guide_crop1_tl.y, 
                                           br_cell_guide_crop1_tl.x, br_cell_guide_crop1_tl.y,
                                           tl_mask, tr_mask, bl_mask, br_mask);
            tl_mask = *(masks1+0);
            tr_mask = *(masks1+1);
            bl_mask = *(masks1+2);
            br_mask = *(masks1+3);

            cv::Mat* masks2 = copy_to_image(tl_cell_guide_crop2, tr_cell_guide_crop2, bl_cell_guide_crop2, br_cell_guide_crop2, 
                                           tl_cell_guide_crop2_tl.x, tl_cell_guide_crop2_tl.y, 
                                           tr_cell_guide_crop2_tl.x, tr_cell_guide_crop2_tl.y,
                                           bl_cell_guide_crop2_tl.x, bl_cell_guide_crop2_tl.y, 
                                           br_cell_guide_crop2_tl.x, br_cell_guide_crop2_tl.y,
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

            cv::imshow("xian_image_cop_process_show:", merge_row1);
            cv::waitKey(1);
            // cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+".jpg", merge_row1);
            
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            std::cout << "FPS: " << 1000.0 / timediff << std::endl;
        } 

        cv::Mat* draw_rectangle(cv::Point tl_xy0, cv::Point tr_xy0, cv::Point bl_xy0, cv::Point br_xy0,
                                cv::Mat tl_image, cv::Mat tr_image, cv::Mat bl_image, cv::Mat br_image, cv::Scalar color)
        {
            static cv::Mat imgs[4];
            int tl_x1 = tl_xy0.x + 256;
            int tl_y1 = tl_xy0.y + 256;
            int tr_x1 = tr_xy0.x + 256;
            int tr_y1 = tr_xy0.y + 256;
            int bl_x1 = bl_xy0.x + 256;
            int bl_y1 = bl_xy0.y + 256;
            int br_x1 = br_xy0.x + 256;
            int br_y1 = br_xy0.y + 256;
            cv::Point tl_xy1 = cv::Point(tl_x1, tl_y1);
            cv::Point tr_xy1 = cv::Point(tr_x1, tr_y1);
            cv::Point bl_xy1 = cv::Point(bl_x1, bl_y1);
            cv::Point br_xy1 = cv::Point(br_x1, br_y1);
            cv::rectangle(tl_image, tl_xy0, tl_xy1, color, 4); 
            cv::rectangle(tr_image, tr_xy0, tr_xy1, color, 4); 
            cv::rectangle(bl_image, bl_xy0, bl_xy1, color, 4); 
            cv::rectangle(br_image, br_xy0, br_xy1, color, 4); 
            imgs[0] = tl_image;
            imgs[1] = tr_image;
            imgs[2] = bl_image;
            imgs[3] = br_image;
            return imgs;
        }

        cv::Mat* copy_to_image(cv::Mat tl_crop_image, cv::Mat tr_crop_image, cv::Mat bl_crop_image, cv::Mat br_crop_image, 
                               int tl_x0, int tl_y0, int tr_x0, int tr_y0, int bl_x0, int bl_y0, int br_x0, int br_y0,
                               cv::Mat tl_mask, cv::Mat tr_mask, cv::Mat bl_mask, cv::Mat br_mask)
        {
            static cv::Mat imgs[4];
            cv::resize(tl_crop_image, tl_crop_image, cv::Size(256, 256), 2);
            tl_mask = zpmc::crop_copy_to_mask(tl_x0, tl_y0, tl_mask, tl_crop_image);

            cv::resize(tr_crop_image, tr_crop_image, cv::Size(256, 256), 2);
            tr_mask = zpmc::crop_copy_to_mask(tr_x0, tr_y0, tr_mask, tr_crop_image);

            cv::resize(bl_crop_image, bl_crop_image, cv::Size(256, 256), 2);
            bl_mask = zpmc::crop_copy_to_mask(bl_x0, bl_y0, bl_mask, bl_crop_image);

            cv::resize(br_crop_image, br_crop_image, cv::Size(256, 256), 2);
            br_mask = zpmc::crop_copy_to_mask(br_x0, br_y0, br_mask, br_crop_image);
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
    ros::init(argc,argv,"xian_get_cell_guide_roi_show_node");
    Xian_GetCellGuideRoiShow xian_get_cell_guide_roi_show;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_get_cell_guide_roi_show.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_GetCellGuideRoiShow::m_timer_HeartBeat_f, &xian_get_cell_guide_roi_show);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}