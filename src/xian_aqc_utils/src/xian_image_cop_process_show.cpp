#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_crop_image_msg.h"
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "zpmc_cv_control.h"


class Xian_ImageCropProcessShow
{
    public:
        Xian_ImageCropProcessShow(): 
            crop_sub(nh, "xian_crop_images", 1),
            images_sub(nh, "xian_aqc_spreader_images", 1),
            sync(MySyncPolicy(10), crop_sub, images_sub) 
        {
            command_publisher_show = nh.advertise<sensor_msgs::Image>("images_crop_process_visualization", 1);
            sync.registerCallback(boost::bind(&Xian_ImageCropProcessShow::command_callback, this, _1, _2));
        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_image_crop_process_show_heart_beat: " << counter << std::endl;
        }

    private:
        ros::NodeHandle nh;
        message_filters::Subscriber<xian_msg_pkg::xian_crop_image_msg_<std::allocator<void>>> crop_sub;
        message_filters::Subscriber<xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>> images_sub;
        typedef message_filters::sync_policies::ExactTime<xian_msg_pkg::xian_crop_image_msg_<std::allocator<void>>,
                                                          xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;
        ros::Publisher command_publisher_show;

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
        sensor_msgs::ImagePtr images_crop_show;

        void command_callback(const boost::shared_ptr<const xian_msg_pkg::xian_crop_image_msg_<std::allocator<void>>>& data,
                              const boost::shared_ptr<const xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>>& images)
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

            tl_image = cv_bridge::toCvShare(images->tl_image, images, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(images->tr_image, images, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(images->bl_image, images, "bgr8")->image;
            br_image = cv_bridge::toCvShare(images->br_image, images, "bgr8")->image;

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

            cv::Mat* container_corner_list = draw_rectangle(container_corner_crop_tl, container_corner_crop_tr, container_corner_crop_bl, container_corner_crop_br,
                                                            tl_image, tr_image, bl_image, br_image, lemon);
            tl_image = *(container_corner_list+0);
            tr_image = *(container_corner_list+1);
            bl_image = *(container_corner_list+2);
            br_image = *(container_corner_list+3);

            cv::Mat* cell_guide_list1 = draw_rectangle(tl_cell_guide_crop1, tr_cell_guide_crop1, bl_cell_guide_crop1, br_cell_guide_crop1,
                                                       tl_image, tr_image, bl_image, br_image, yellow);
            tl_image = *(cell_guide_list1+0);
            tr_image = *(cell_guide_list1+1);
            bl_image = *(cell_guide_list1+2);
            br_image = *(cell_guide_list1+3);

            cv::Mat* cell_guide_list2 = draw_rectangle(tl_cell_guide_crop2, tr_cell_guide_crop2, bl_cell_guide_crop2, br_cell_guide_crop2,
                                                       tl_image, tr_image, bl_image, br_image, blue);
            tl_image = *(cell_guide_list2+0);
            tr_image = *(cell_guide_list2+1);
            bl_image = *(cell_guide_list2+2);
            br_image = *(cell_guide_list2+3);


            cv::Mat tl_container_corner_crop_image = cv_bridge::toCvShare(data->tl_container_corner_crop_image, data, "bgr8")->image; 
            cv::Mat tr_container_corner_crop_image = cv_bridge::toCvShare(data->tr_container_corner_crop_image, data, "bgr8")->image; 
            cv::Mat bl_container_corner_crop_image = cv_bridge::toCvShare(data->bl_container_corner_crop_image, data, "bgr8")->image;
            cv::Mat br_container_corner_crop_image = cv_bridge::toCvShare(data->br_container_corner_crop_image, data, "bgr8")->image;

            cv::Mat tl_cell_guide_crop_image1 = cv_bridge::toCvShare(data->tl_cell_guide_crop_image1, data, "bgr8")->image; 
            cv::Mat tr_cell_guide_crop_image1 = cv_bridge::toCvShare(data->tr_cell_guide_crop_image1, data, "bgr8")->image; 
            cv::Mat bl_cell_guide_crop_image1 = cv_bridge::toCvShare(data->bl_cell_guide_crop_image1, data, "bgr8")->image;
            cv::Mat br_cell_guide_crop_image1 = cv_bridge::toCvShare(data->br_cell_guide_crop_image1, data, "bgr8")->image;

            cv::Mat tl_cell_guide_crop_image2 = cv_bridge::toCvShare(data->tl_cell_guide_crop_image2, data, "bgr8")->image; 
            cv::Mat tr_cell_guide_crop_image2 = cv_bridge::toCvShare(data->tr_cell_guide_crop_image2, data, "bgr8")->image; 
            cv::Mat bl_cell_guide_crop_image2 = cv_bridge::toCvShare(data->bl_cell_guide_crop_image2, data, "bgr8")->image;
            cv::Mat br_cell_guide_crop_image2 = cv_bridge::toCvShare(data->br_cell_guide_crop_image2, data, "bgr8")->image;

            cv::Mat tl_mask = cv::Mat::zeros(tl_image.size(),tl_image.type());
            cv::Mat tr_mask = cv::Mat::zeros(tr_image.size(),tr_image.type());
            cv::Mat bl_mask = cv::Mat::zeros(bl_image.size(),tl_image.type());
            cv::Mat br_mask = cv::Mat::zeros(br_image.size(),tr_image.type());

            cv::resize(tl_container_corner_crop_image, tl_container_corner_crop_image, cv::Size(256, 256), 2);
            tl_mask = zpmc::crop_copy_to_mask(container_corner_tl_x0, container_corner_tl_y0, tl_mask, tl_container_corner_crop_image);

            cv::resize(tr_container_corner_crop_image, tr_container_corner_crop_image, cv::Size(256, 256), 2);
            tr_mask = zpmc::crop_copy_to_mask(container_corner_tr_x0, container_corner_tr_y0, tr_mask, tr_container_corner_crop_image);

            cv::resize(bl_container_corner_crop_image, bl_container_corner_crop_image, cv::Size(256, 256), 2);
            bl_mask = zpmc::crop_copy_to_mask(container_corner_bl_x0, container_corner_bl_y0, bl_mask, bl_container_corner_crop_image);

            cv::resize(br_container_corner_crop_image, br_container_corner_crop_image, cv::Size(256, 256), 2);
            br_mask = zpmc::crop_copy_to_mask(container_corner_br_x0, container_corner_br_y0, br_mask, br_container_corner_crop_image);


            cv::resize(tl_cell_guide_crop_image1, tl_cell_guide_crop_image1, cv::Size(256, 256), 2);
            tl_mask = zpmc::crop_copy_to_mask(clip1_cell_guide_tl_x, clip1_cell_guide_tl_y, tl_mask, tl_cell_guide_crop_image1);

            cv::resize(tr_cell_guide_crop_image1, tr_cell_guide_crop_image1, cv::Size(256, 256), 2);
            tr_mask = zpmc::crop_copy_to_mask(clip1_cell_guide_tr_x, clip1_cell_guide_tr_y, tr_mask, tr_cell_guide_crop_image1);

            cv::resize(bl_cell_guide_crop_image1, bl_cell_guide_crop_image1, cv::Size(256, 256), 2);
            bl_mask = zpmc::crop_copy_to_mask(clip1_cell_guide_bl_x, clip1_cell_guide_bl_y, bl_mask, bl_cell_guide_crop_image1);

            cv::resize(br_cell_guide_crop_image1, br_cell_guide_crop_image1, cv::Size(256, 256), 2);
            br_mask = zpmc::crop_copy_to_mask(clip1_cell_guide_br_x, clip1_cell_guide_br_y, br_mask, br_cell_guide_crop_image1);


            cv::resize(tl_cell_guide_crop_image2, tl_cell_guide_crop_image2, cv::Size(256, 256), 2);
            tl_mask = zpmc::crop_copy_to_mask(clip2_cell_guide_tl_x, clip2_cell_guide_tl_y, tl_mask, tl_cell_guide_crop_image2);

            cv::resize(tr_cell_guide_crop_image2, tr_cell_guide_crop_image2, cv::Size(256, 256), 2);
            tr_mask = zpmc::crop_copy_to_mask(clip2_cell_guide_tr_x, clip2_cell_guide_tr_y, tr_mask, tr_cell_guide_crop_image2);

            cv::resize(bl_cell_guide_crop_image2, bl_cell_guide_crop_image2, cv::Size(256, 256), 2);
            bl_mask = zpmc::crop_copy_to_mask(clip2_cell_guide_bl_x, clip2_cell_guide_bl_y, bl_mask, bl_cell_guide_crop_image2);

            cv::resize(br_cell_guide_crop_image2, br_cell_guide_crop_image2, cv::Size(256, 256), 2);
            br_mask = zpmc::crop_copy_to_mask(clip2_cell_guide_br_x, clip2_cell_guide_br_y, br_mask, br_cell_guide_crop_image2);

            
            cv::Mat mask_merge_col_0 = zpmc::zpmc_images_merge_row(tl_mask, bl_mask);
            cv::Mat mask_merge_col_1 = zpmc::zpmc_images_merge_row(tr_mask, br_mask);
            
            cv::Mat src_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            cv::Mat src_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            cv::Mat merge_log = zpmc::zpmc_images_merge_col(src_merge_col_0, src_merge_col_1);

            cv::Mat merge_row0 = zpmc::zpmc_images_merge_col(mask_merge_col_0, merge_log);
            cv::Mat merge_row1 = zpmc::zpmc_images_merge_col(merge_row0, mask_merge_col_1);


            cv::resize(merge_row1, merge_row1, cv::Size(merge_log.cols/2, merge_log.rows/4), 2); 

            // cv::imshow("xian_image_cop_process_show:", merge_row1);
            // cv::waitKey(1);
            // cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+".jpg", merge_row1);
            images_crop_show = cv_bridge::CvImage(std_msgs::Header(), "bgr8", merge_row1).toImageMsg();
            command_publisher_show.publish(images_crop_show);

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