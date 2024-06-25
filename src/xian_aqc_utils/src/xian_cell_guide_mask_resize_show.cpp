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
#include "xian_msg_pkg/xian_cell_guide_mask_resize.h"
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "zpmc_cv_control.h"


class Xian_CellGuideMaskResizeShow
{
    public:
        Xian_CellGuideMaskResizeShow(): 
            cell_guide_mask_resize_sub(nh, "xian_cell_guide_masks_resize", 1),
            images_sub(nh, "xian_spreader_image_align_with_cell_guide_crop", 1),
            sync(MySyncPolicy(10), cell_guide_mask_resize_sub, images_sub) 
        {
            command_publisher_show = nh.advertise<sensor_msgs::Image>("cell_guide_masks_resize_visualization", 1);
            sync.registerCallback(boost::bind(&Xian_CellGuideMaskResizeShow::command_callback, this, _1, _2));
        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_cell_guide_mask_resize_show_heart_beat: " << counter << std::endl;
        }

    private:
        ros::NodeHandle nh;
        message_filters::Subscriber<xian_msg_pkg::xian_cell_guide_mask_resize_<std::allocator<void>>> cell_guide_mask_resize_sub;
        message_filters::Subscriber<xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>> images_sub;
        typedef message_filters::sync_policies::ExactTime<xian_msg_pkg::xian_cell_guide_mask_resize_<std::allocator<void>>,
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

        int tl_container_corner_cx = 0;
        int tl_container_corner_cy = 0;
        int tr_container_corner_cx = 0;
        int tr_container_corner_cy = 0;
        int bl_container_corner_cx = 0;
        int bl_container_corner_cy = 0;
        int br_container_corner_cx = 0;
        int br_container_corner_cy = 0;

        int tl_cell_guide_crop_tl_x = 0;
        int tl_cell_guide_crop_tl_y = 0;
        int tr_cell_guide_crop_tl_x = 0;
        int tr_cell_guide_crop_tl_y = 0;
        int bl_cell_guide_crop_tl_x = 0;
        int bl_cell_guide_crop_tl_y = 0;
        int br_cell_guide_crop_tl_x = 0;
        int br_cell_guide_crop_tl_y = 0;

        cv::Point tl_container_corner = cv::Point(0,0);
        cv::Point tr_container_corner = cv::Point(0,0);
        cv::Point bl_container_corner = cv::Point(0,0);
        cv::Point br_container_corner = cv::Point(0,0);

        cv::Point tl_cell_guide_corner = cv::Point(0,0);
        cv::Point tr_cell_guide_corner = cv::Point(0,0);
        cv::Point bl_cell_guide_corner = cv::Point(0,0);
        cv::Point br_cell_guide_corner = cv::Point(0,0);
 
        cv::Mat tl_image, tr_image, bl_image, br_image;
        cv::Mat tl_mask_resize;
        cv::Mat tr_mask_resize;
        cv::Mat bl_mask_resize;
        cv::Mat br_mask_resize;

        sensor_msgs::ImagePtr cell_guide_masks_resize_show;

        void command_callback(const boost::shared_ptr<const xian_msg_pkg::xian_cell_guide_mask_resize_<std::allocator<void>>>& data,
                              const boost::shared_ptr<const xian_msg_pkg::xian_spreader_images_msg_<std::allocator<void>>>& images)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();

            tl_image = cv_bridge::toCvShare(images->tl_image, images, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(images->tr_image, images, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(images->bl_image, images, "bgr8")->image;
            br_image = cv_bridge::toCvShare(images->br_image, images, "bgr8")->image;

            tl_mask_resize = cv_bridge::toCvShare(data->tl_mask_resize, data, "bgr8")->image;
            tr_mask_resize = cv_bridge::toCvShare(data->tr_mask_resize, data, "bgr8")->image;
            bl_mask_resize = cv_bridge::toCvShare(data->bl_mask_resize, data, "bgr8")->image;
            br_mask_resize = cv_bridge::toCvShare(data->br_mask_resize, data, "bgr8")->image;

            
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

            tl_cell_guide_crop_tl_x = data->tl_cell_guide_crop_tl_x;
            tl_cell_guide_crop_tl_y = data->tl_cell_guide_crop_tl_y;
            tr_cell_guide_crop_tl_x = data->tr_cell_guide_crop_tl_x;
            tr_cell_guide_crop_tl_y = data->tr_cell_guide_crop_tl_y;
            bl_cell_guide_crop_tl_x = data->bl_cell_guide_crop_tl_x;
            bl_cell_guide_crop_tl_y = data->bl_cell_guide_crop_tl_y;
            br_cell_guide_crop_tl_x = data->br_cell_guide_crop_tl_x;
            br_cell_guide_crop_tl_y = data->br_cell_guide_crop_tl_y;

            tl_cell_guide_corner.x = tl_cell_guide_crop_tl_x+128;
            tl_cell_guide_corner.y = tl_cell_guide_crop_tl_y+128;
            tr_cell_guide_corner.x = tr_cell_guide_crop_tl_x+128;
            tr_cell_guide_corner.y = tr_cell_guide_crop_tl_y+128;
            bl_cell_guide_corner.x = bl_cell_guide_crop_tl_x+128;
            bl_cell_guide_corner.y = bl_cell_guide_crop_tl_y+128;
            br_cell_guide_corner.x = br_cell_guide_crop_tl_x+128;
            br_cell_guide_corner.y = br_cell_guide_crop_tl_y+128;


            cv::circle(tl_image, tl_container_corner, 16, red, -1);
            cv::circle(tr_image, tr_container_corner, 16, red, -1);
            cv::circle(bl_image, bl_container_corner, 16, red, -1);
            cv::circle(br_image, br_container_corner, 16, red, -1);

            cv::circle(tl_image, tl_cell_guide_corner, 16, blue, -1);
            cv::circle(tr_image, tr_cell_guide_corner, 16, blue, -1);
            cv::circle(bl_image, bl_cell_guide_corner, 16, blue, -1);
            cv::circle(br_image, br_cell_guide_corner, 16, blue, -1);

            // printf("tl_cell_guide_corner: %d, %d \n", tl_cell_guide_corner.x, tl_cell_guide_corner.y);
            // printf("tr_cell_guide_corner: %d, %d \n", tr_cell_guide_corner.x, tr_cell_guide_corner.y);
            // printf("bl_cell_guide_corner: %d, %d \n", bl_cell_guide_corner.x, bl_cell_guide_corner.y);
            // printf("br_cell_guide_corner: %d, %d \n", br_cell_guide_corner.x, br_cell_guide_corner.y);

            cv::Mat tl_mask = cv::Mat::zeros(tl_image.size(),tl_image.type());
            cv::Mat tr_mask = cv::Mat::zeros(tr_image.size(),tr_image.type());
            cv::Mat bl_mask = cv::Mat::zeros(bl_image.size(),tl_image.type());
            cv::Mat br_mask = cv::Mat::zeros(br_image.size(),tr_image.type());

            cv::Mat* masks = copy_to_image(tl_mask_resize, tr_mask_resize, 
                                            bl_mask_resize, br_mask_resize, 
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

            cv::Mat merge_row0 = zpmc::zpmc_images_merge_col(mask_merge_col_0, merge_log);
            cv::Mat merge_row1 = zpmc::zpmc_images_merge_col(merge_row0, mask_merge_col_1);


            cv::resize(merge_row1, merge_row1, cv::Size(merge_log.cols/2, merge_log.rows/4), 2); 

            // cv::imshow("xian_cell_guide_mask_resize_show:", merge_row1);
            // cv::waitKey(1);
            // cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+".jpg", merge_row1);
            cell_guide_masks_resize_show = cv_bridge::CvImage(std_msgs::Header(), "bgr8", merge_row1).toImageMsg();
            command_publisher_show.publish(cell_guide_masks_resize_show);
            
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            std::cout << "FPS: " << 1000.0 / timediff << std::endl;
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
    ros::init(argc,argv,"xian_cell_guide_mask_resize_show_node");
    Xian_CellGuideMaskResizeShow xian_cell_guide_mask_resize_show;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_cell_guide_mask_resize_show.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_CellGuideMaskResizeShow::m_timer_HeartBeat_f, &xian_cell_guide_mask_resize_show);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}