#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

// #include <opencv2/cudaarithm.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/cudawarping.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_cell_guide_point_identification.h"
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "xian_msg_pkg/xian_cell_guide_msg.h"
#include "zpmc_cv_control.h"


class Xian_CellGuidePointIdentificationShow
{
    public:
        Xian_CellGuidePointIdentificationShow(): 
            cell_guide_points_sub(nh, "xian_cell_guide_points", 1),
            images_sub(nh, "xian_spreader_image_align_with_cell_guide_crop", 1),
            sync(MySyncPolicy(10), cell_guide_points_sub, images_sub) 
        {
            command_publisher_show = nh.advertise<sensor_msgs::Image>("xian_visualization", 1);
            command_publisher_results = nh.advertise<xian_msg_pkg::xian_cell_guide_msg>("xian_results", 1);
            sync.registerCallback(boost::bind(&Xian_CellGuidePointIdentificationShow::command_callback, this, _1, _2));
        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_cell_guide_point_identification_show_heart_beat: " << counter << std::endl;
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher command_publisher_show;
        ros::Publisher command_publisher_results;
        message_filters::Subscriber<xian_msg_pkg::xian_cell_guide_point_identification_<std::allocator<void>>> cell_guide_points_sub;
        message_filters::Subscriber<xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>> images_sub;
        typedef message_filters::sync_policies::ExactTime<xian_msg_pkg::xian_cell_guide_point_identification_<std::allocator<void>>,
                                                          xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;

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

        int tl_cell_guide_cx = 0;
        int tl_cell_guide_cy = 0;
        int tr_cell_guide_cx = 0;
        int tr_cell_guide_cy = 0;
        int bl_cell_guide_cx = 0;
        int bl_cell_guide_cy = 0;
        int br_cell_guide_cx = 0;
        int br_cell_guide_cy = 0;

        int tl_cell_guide_crop_tl_x = 0;
        int tl_cell_guide_crop_tl_y = 0;
        int tr_cell_guide_crop_tl_x = 0;
        int tr_cell_guide_crop_tl_y = 0;
        int bl_cell_guide_crop_tl_x = 0;
        int bl_cell_guide_crop_tl_y = 0;
        int br_cell_guide_crop_tl_x = 0;
        int br_cell_guide_crop_tl_y = 0;

        // 分别计算 "检测点" 与 "目标点" 的偏差
        double tl_ex = 0;
        double tr_ex = 0;
        double bl_ex = 0;
        double br_ex = 0;
        double tl_ey = 0;
        double tr_ey = 0;
        double bl_ey = 0;
        double br_ey = 0;

        bool is_tl_stand = true;
        bool is_tr_stand = true;
        bool is_bl_stand = true;
        bool is_br_stand = true;
        int num_detect_points = 4;
        zpmc::zpmc_SpreaderLogicControl zpmc_spreader_logic_control;
        double* spreader_control_err;
        double zpmc_gantry_pixel_error = 0;
        double zpmc_trolley_pixel_error = 0;
        double zpmc_rotate_pixel_error = 0;
        int error_code = 9000;

        int retract_box_state0 = 0;
        int retract_box_state1 = 0;
        int retract_box_state2 = 0;
        int retract_box_state3 = 0;
        int retract_box_mode0  = 0;
        int retract_box_mode1  = 0;
        int retract_box_mode2  = 0;
        int retract_box_mode3  = 0;

        cv::Point tl_container_corner = cv::Point(0,0);
        cv::Point tr_container_corner = cv::Point(0,0);
        cv::Point bl_container_corner = cv::Point(0,0);
        cv::Point br_container_corner = cv::Point(0,0);

        cv::Point tl_cell_guide_c = cv::Point(0,0);
        cv::Point tr_cell_guide_c = cv::Point(0,0);
        cv::Point bl_cell_guide_c = cv::Point(0,0);
        cv::Point br_cell_guide_c = cv::Point(0,0);

        cv::Point tl_cell_guide_c_target = cv::Point(0,0);
        cv::Point tr_cell_guide_c_target = cv::Point(0,0);
        cv::Point bl_cell_guide_c_target = cv::Point(0,0);
        cv::Point br_cell_guide_c_target = cv::Point(0,0);
 
        cv::Mat tl_image, tr_image, bl_image, br_image;
        cv::Mat tl_mask_resize_show;
        cv::Mat tr_mask_resize_show;
        cv::Mat bl_mask_resize_show;
        cv::Mat br_mask_resize_show;

        sensor_msgs::ImagePtr final_visualization;
        xian_msg_pkg::xian_cell_guide_msg final_results;
        // cv::Mat merge_resize;
        // cv::cuda::GpuMat merge_gpu;
        // cv::cuda::GpuMat merge_resize_gpu;

        cv::Mat right_icon = cv::imread("/root/code/xian_aqc_ws/xian_project_file/icon/right.png");
        cv::Mat left_icon = cv::imread("/root/code/xian_aqc_ws/xian_project_file/icon/left.png");
        cv::Mat shun_icon = cv::imread("/root/code/xian_aqc_ws/xian_project_file/icon/shun.png");
        cv::Mat ni_icon = cv::imread("/root/code/xian_aqc_ws/xian_project_file/icon/ni.png");
        cv::Mat top_icon = cv::imread("/root/code/xian_aqc_ws/xian_project_file/icon/top.png");
        cv::Mat bottom_icon = cv::imread("/root/code/xian_aqc_ws/xian_project_file/icon/bottom.png");


        void command_callback(const boost::shared_ptr<const xian_msg_pkg::xian_cell_guide_point_identification_<std::allocator<void>>>& data,
                              const boost::shared_ptr<const xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>>& images)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            std::cout << "Time:" << timeStr << std::endl;

            tl_image = cv_bridge::toCvShare(images->tl_image, images, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(images->tr_image, images, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(images->bl_image, images, "bgr8")->image;
            br_image = cv_bridge::toCvShare(images->br_image, images, "bgr8")->image;

            tl_mask_resize_show = cv_bridge::toCvShare(data->tl_mask_resize_show, data, "bgr8")->image; 
            tr_mask_resize_show = cv_bridge::toCvShare(data->tr_mask_resize_show, data, "bgr8")->image; 
            bl_mask_resize_show = cv_bridge::toCvShare(data->bl_mask_resize_show, data, "bgr8")->image;
            br_mask_resize_show = cv_bridge::toCvShare(data->br_mask_resize_show, data, "bgr8")->image;
            
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

            tl_cell_guide_cx = data->tl_cell_guide_cx;
            tl_cell_guide_cy = data->tl_cell_guide_cy;
            tr_cell_guide_cx = data->tr_cell_guide_cx;
            tr_cell_guide_cy = data->tr_cell_guide_cy;
            bl_cell_guide_cx = data->bl_cell_guide_cx;
            bl_cell_guide_cy = data->bl_cell_guide_cy;
            br_cell_guide_cx = data->br_cell_guide_cx;
            br_cell_guide_cy = data->br_cell_guide_cy;

            tl_cell_guide_c.x = tl_cell_guide_cx;
            tl_cell_guide_c.y = tl_cell_guide_cy;
            tr_cell_guide_c.x = tr_cell_guide_cx;
            tr_cell_guide_c.y = tr_cell_guide_cy;
            bl_cell_guide_c.x = bl_cell_guide_cx;
            bl_cell_guide_c.y = bl_cell_guide_cy;
            br_cell_guide_c.x = br_cell_guide_cx;
            br_cell_guide_c.y = br_cell_guide_cy;

            tl_cell_guide_crop_tl_x = data->tl_cell_guide_crop_tl_x;
            tl_cell_guide_crop_tl_y = data->tl_cell_guide_crop_tl_y;
            tr_cell_guide_crop_tl_x = data->tr_cell_guide_crop_tl_x;
            tr_cell_guide_crop_tl_y = data->tr_cell_guide_crop_tl_y;
            bl_cell_guide_crop_tl_x = data->bl_cell_guide_crop_tl_x;
            bl_cell_guide_crop_tl_y = data->bl_cell_guide_crop_tl_y;
            br_cell_guide_crop_tl_x = data->br_cell_guide_crop_tl_x;
            br_cell_guide_crop_tl_y = data->br_cell_guide_crop_tl_y;


            tl_cell_guide_c_target.x = tl_cell_guide_c.x+tl_cell_guide_crop_tl_x;
            tl_cell_guide_c_target.y = tl_cell_guide_c.y+tl_cell_guide_crop_tl_y;
            tr_cell_guide_c_target.x = tr_cell_guide_c.x+tr_cell_guide_crop_tl_x;
            tr_cell_guide_c_target.y = tr_cell_guide_c.y+tr_cell_guide_crop_tl_y;
            bl_cell_guide_c_target.x = bl_cell_guide_c.x+bl_cell_guide_crop_tl_x;
            bl_cell_guide_c_target.y = bl_cell_guide_c.y+bl_cell_guide_crop_tl_y;
            br_cell_guide_c_target.x = br_cell_guide_c.x+br_cell_guide_crop_tl_x;
            br_cell_guide_c_target.y = br_cell_guide_c.y+br_cell_guide_crop_tl_y;

            // ------------------------------------------------ error -------------------------------------------------
            // 单张图像上的偏差计算
            tl_ex = tl_cell_guide_c_target.x - tl_container_corner.x; 
            tl_ey = tl_cell_guide_c_target.y - tl_container_corner.y; 
            tr_ex = tr_cell_guide_c_target.x - tr_container_corner.x; 
            tr_ey = tr_cell_guide_c_target.y - tr_container_corner.y; 
            bl_ex = bl_cell_guide_c_target.x - bl_container_corner.x; 
            bl_ey = bl_cell_guide_c_target.y - bl_container_corner.y; 
            br_ex = br_cell_guide_c_target.x - br_container_corner.x; 
            br_ey = br_cell_guide_c_target.y - br_container_corner.y; 

            // 判断是否检测到
            // ----------------------------判断检测的点是否为标准箱角线的角点--------------------------------
            is_tl_stand = (tl_cell_guide_c.x != -1 && tl_cell_guide_c.y != -1);
            is_tr_stand = (tr_cell_guide_c.x != -1 && tr_cell_guide_c.y != -1);
            is_bl_stand = (bl_cell_guide_c.x != -1 && bl_cell_guide_c.y != -1);
            is_br_stand = (br_cell_guide_c.x != -1 && br_cell_guide_c.y != -1);
            num_detect_points = is_tl_stand + is_tr_stand + is_bl_stand + is_br_stand;


            if(num_detect_points < 3)
            {
                error_code = 9003;
            }
            else
            {
                error_code = 9000;
            }

            if(num_detect_points == 4)
            {
                spreader_control_err = zpmc_spreader_logic_control.zpmc_4points_spreader_eror(tl_ex, tr_ex, bl_ex, br_ex, tl_ey, tr_ey, bl_ey, br_ey);
                zpmc_gantry_pixel_error = *(spreader_control_err + 1);
                zpmc_trolley_pixel_error = *(spreader_control_err + 0);
                zpmc_rotate_pixel_error = *(spreader_control_err + 2);
                
            }
            else if(num_detect_points == 3)
            {
                if(is_tl_stand == false)
                {
                    spreader_control_err = zpmc_spreader_logic_control.zpmc_3points_spreader_eror(0, tl_ex, tr_ex, bl_ex, br_ex, tl_ey, tr_ey, bl_ey, br_ey);
                    zpmc_gantry_pixel_error = *(spreader_control_err + 1);
                    zpmc_trolley_pixel_error = *(spreader_control_err + 0);
                    zpmc_rotate_pixel_error = *(spreader_control_err + 2);
                }
                else if(is_tr_stand == false)
                {
                    spreader_control_err = zpmc_spreader_logic_control.zpmc_3points_spreader_eror(1, tl_ex, tr_ex, bl_ex, br_ex, tl_ey, tr_ey, bl_ey, br_ey);
                    zpmc_gantry_pixel_error = *(spreader_control_err + 1);
                    zpmc_trolley_pixel_error = *(spreader_control_err + 0);
                    zpmc_rotate_pixel_error = *(spreader_control_err + 2);
                }
                else if(is_bl_stand == false)
                {
                    spreader_control_err = zpmc_spreader_logic_control.zpmc_3points_spreader_eror(2, tl_ex, tr_ex, bl_ex, br_ex, tl_ey, tr_ey, bl_ey, br_ey);
                    zpmc_gantry_pixel_error = *(spreader_control_err + 1);
                    zpmc_trolley_pixel_error = *(spreader_control_err + 0);
                    zpmc_rotate_pixel_error = *(spreader_control_err + 2);
                }
                else if(is_br_stand == false)
                {
                    spreader_control_err = zpmc_spreader_logic_control.zpmc_3points_spreader_eror(3, tl_ex, tr_ex, bl_ex, br_ex, tl_ey, tr_ey, bl_ey, br_ey);
                    zpmc_gantry_pixel_error = *(spreader_control_err + 1);
                    zpmc_trolley_pixel_error = *(spreader_control_err + 0);
                    zpmc_rotate_pixel_error = *(spreader_control_err + 2);
                }
                else
                {
                    zpmc_gantry_pixel_error = 0;
                    zpmc_trolley_pixel_error = 0;
                    zpmc_rotate_pixel_error = 0;
                }
            }
            else
            {
                // 检测数量小于3个点时，报错。并将大车、小车、旋转方向置为0
                zpmc_gantry_pixel_error = 0;
                zpmc_trolley_pixel_error = 0;
                zpmc_rotate_pixel_error = 0;
            }


            // ------------------------------------------------ show --------------------------------------------------
            cv::circle(tl_image, tl_container_corner, 16, red, -1);
            cv::circle(tr_image, tr_container_corner, 16, red, -1);
            cv::circle(bl_image, bl_container_corner, 16, red, -1);
            cv::circle(br_image, br_container_corner, 16, red, -1);

            if(tl_cell_guide_c.x == -1 && tl_cell_guide_c.y == -1)
            {
                cv::putText(tl_image,"Can't detected target point!",cv::Point(40, 560), cv::FONT_HERSHEY_SIMPLEX, 2, blue,5,8);
            }
            else
            {
                cv::circle(tl_image, tl_cell_guide_c_target, 16, yellow, -1);
            }

            if(tr_cell_guide_c.x == -1 && tr_cell_guide_c.y == -1)
            {
                cv::putText(tr_image,"Can't detected target point!",cv::Point(40, 560), cv::FONT_HERSHEY_SIMPLEX, 2, blue,5,8);
            }
            else
            {
                cv::circle(tr_image, tr_cell_guide_c_target, 16, yellow, -1);
            }

            if(bl_cell_guide_c.x == -1 && bl_cell_guide_c.y == -1)
            {
                cv::putText(bl_image,"Can't detected target point!",cv::Point(40, 560), cv::FONT_HERSHEY_SIMPLEX, 2, blue,5,8);
            }
            else
            {
                cv::circle(bl_image, bl_cell_guide_c_target, 16, yellow, -1);
            }

            if(br_cell_guide_c.x == -1 && br_cell_guide_c.y == -1)
            {
                cv::putText(br_image,"Can't detected target point!",cv::Point(40, 560), cv::FONT_HERSHEY_SIMPLEX, 2, blue,5,8);
            }
            else
            {
                cv::circle(br_image, br_cell_guide_c_target, 16, yellow, -1);

            }

            cv::Mat tl_mask = cv::Mat::zeros(tl_image.size(),tl_image.type());
            cv::Mat tr_mask = cv::Mat::zeros(tr_image.size(),tr_image.type());
            cv::Mat bl_mask = cv::Mat::zeros(bl_image.size(),tl_image.type());
            cv::Mat br_mask = cv::Mat::zeros(br_image.size(),tr_image.type());

            cv::Mat* masks = copy_to_image(tl_mask_resize_show, tr_mask_resize_show, 
                                           bl_mask_resize_show, br_mask_resize_show, 
                                           tl_cell_guide_crop_tl_x, tl_cell_guide_crop_tl_y,
                                           tr_cell_guide_crop_tl_x, tr_cell_guide_crop_tl_y,
                                           bl_cell_guide_crop_tl_x, bl_cell_guide_crop_tl_y,
                                           br_cell_guide_crop_tl_x, br_cell_guide_crop_tl_y,
                                           tl_mask, tr_mask, bl_mask, br_mask);
            tl_mask = *(masks+0);
            tr_mask = *(masks+1);
            bl_mask = *(masks+2);
            br_mask = *(masks+3);
            
            cv::Mat mask_merge_col_0 = zpmc::zpmc_images_merge_row(tl_mask, bl_mask);
            cv::Mat mask_merge_col_1 = zpmc::zpmc_images_merge_row(tr_mask, br_mask);
            
            cv::Mat src_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            cv::Mat src_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            cv::Mat merge_log = zpmc::zpmc_images_merge_col(src_merge_col_0, src_merge_col_1);

            if(zpmc_gantry_pixel_error > 0)
            {
                merge_log = zpmc::crop_copy_to_mask(tl_mask.cols-right_icon.cols/2, tl_mask.rows-right_icon.rows/2, merge_log, right_icon);
            }
            else if(zpmc_gantry_pixel_error < 0)
            {
                merge_log = zpmc::crop_copy_to_mask(tl_mask.cols-right_icon.cols/2, tl_mask.rows-right_icon.rows/2, merge_log, left_icon);
            }

            if(zpmc_trolley_pixel_error > 0)
            {
                merge_log = zpmc::crop_copy_to_mask(tl_mask.cols-bottom_icon.cols/2, tl_mask.rows+200, merge_log, bottom_icon);
            }
            else if(zpmc_trolley_pixel_error < 0)
            {
                merge_log = zpmc::crop_copy_to_mask(tl_mask.cols-top_icon.cols/2, tl_mask.rows+200, merge_log, top_icon);
            }

            if(zpmc_rotate_pixel_error > 0)
            {
                merge_log = zpmc::crop_copy_to_mask(tl_mask.cols-ni_icon.cols/2, tl_mask.rows-200, merge_log, ni_icon);
            }
            else if(zpmc_rotate_pixel_error < 0)
            {
                merge_log = zpmc::crop_copy_to_mask(tl_mask.cols-shun_icon.cols/2, tl_mask.rows-200, merge_log, shun_icon);
            }

            cv::Mat merge_row0 = zpmc::zpmc_images_merge_col(mask_merge_col_0, merge_log);
            cv::Mat merge_row1 = zpmc::zpmc_images_merge_col(merge_row0, mask_merge_col_1);
            // merge_gpu.upload(merge_row1);
            // cv::cuda::resize(merge_gpu, merge_resize_gpu, cv::Size(merge_log.cols/2, merge_log.rows/4), 2);
            // merge_resize_gpu.download(merge_resize);
            cv::resize(merge_row1, merge_row1, cv::Size(merge_log.cols/2, merge_log.rows/4), 2); 

            // cv::imshow("xian_cell_guide_point_identification_show:", merge_row1);
            // cv::waitKey(1);
            // cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+".jpg", merge_row1);


            // ------------------------------------------------ publish -----------------------------------------
            final_visualization = cv_bridge::CvImage(std_msgs::Header(), "bgr8", merge_row1).toImageMsg();
            command_publisher_show.publish(final_visualization);

            final_results.xian_container_corner_point_tl_x = data->tl_container_corner_cx;
            final_results.xian_container_corner_point_tl_y = data->tl_container_corner_cy;
            final_results.xian_container_corner_point_tr_x = data->tr_container_corner_cx;
            final_results.xian_container_corner_point_tr_y = data->tr_container_corner_cy;
            final_results.xian_container_corner_point_bl_x = data->bl_container_corner_cx;
            final_results.xian_container_corner_point_bl_y = data->bl_container_corner_cy;
            final_results.xian_container_corner_point_br_x = data->br_container_corner_cx;
            final_results.xian_container_corner_point_br_y = data->br_container_corner_cy;
            final_results.xian_cell_guide_point_tl_x = tl_cell_guide_c_target.x;
            final_results.xian_cell_guide_point_tl_y = tl_cell_guide_c_target.y;
            final_results.xian_cell_guide_point_tr_x = tr_cell_guide_c_target.x;
            final_results.xian_cell_guide_point_tr_y = tr_cell_guide_c_target.y;
            final_results.xian_cell_guide_point_bl_x = bl_cell_guide_c_target.x;
            final_results.xian_cell_guide_point_bl_y = bl_cell_guide_c_target.y;
            final_results.xian_cell_guide_point_br_x = br_cell_guide_c_target.x;
            final_results.xian_cell_guide_point_br_y = br_cell_guide_c_target.y;
            final_results.xian_gantry_pixel_error = zpmc_gantry_pixel_error;
            final_results.xian_trolley_pixel_error = zpmc_trolley_pixel_error;
            final_results.xian_rotate_pixel_error = zpmc_rotate_pixel_error;
            final_results.error_code = error_code;

            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state0", retract_box_state0);  
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state1", retract_box_state1);  
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state2", retract_box_state2);  
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state3", retract_box_state3);
            final_results.retract_box_state0 = retract_box_state0;
            final_results.retract_box_state1 = retract_box_state1;
            final_results.retract_box_state2 = retract_box_state2;
            final_results.retract_box_state3 = retract_box_state3;

            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode0", retract_box_mode0);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode1", retract_box_mode1);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode2", retract_box_mode2);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode3", retract_box_mode3);
            final_results.retract_box_mode0 = retract_box_mode0;
            final_results.retract_box_mode1 = retract_box_mode1;
            final_results.retract_box_mode2 = retract_box_mode2;
            final_results.retract_box_mode3 = retract_box_mode3;

            command_publisher_results.publish(final_results);


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