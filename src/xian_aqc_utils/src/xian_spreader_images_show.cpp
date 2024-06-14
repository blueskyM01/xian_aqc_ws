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

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_spreader_images_msg.h"
#include "zpmc_cv_control.h"


class Xian_SpreaderImagesShow
{
    public:
        Xian_SpreaderImagesShow()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_spreader_images_msg>("xian_aqc_spreader_images", 1, &Xian_SpreaderImagesShow::command_callback, this);

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_spreader_images_show_heart_beat: " << counter << std::endl;
        }

    private:
        ros::Subscriber command_subscribe_;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        std::string timeStr;
 
        cv::Mat tl_image, tr_image, bl_image, br_image, merge_row1_resize;
        sensor_msgs::Image spreder_imgs_resize;

        void command_callback(const xian_msg_pkg::xian_spreader_images_msgConstPtr& data)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();

            tl_image = cv_bridge::toCvShare(data->tl_image, data, "bgr8")->image; 
            tr_image = cv_bridge::toCvShare(data->tr_image, data, "bgr8")->image; 
            bl_image = cv_bridge::toCvShare(data->bl_image, data, "bgr8")->image;
            br_image = cv_bridge::toCvShare(data->br_image, data, "bgr8")->image;

            cv::Mat mask_merge_col_0 = zpmc::zpmc_images_merge_row(tl_image, bl_image);
            cv::Mat mask_merge_col_1 = zpmc::zpmc_images_merge_row(tr_image, br_image);
            
            cv::Mat merge_row1 = zpmc::zpmc_images_merge_col(mask_merge_col_0, mask_merge_col_1);

            // cv::cuda::GpuMat merge_row1_gpu, merge_row1_resize_gpu;
            // merge_row1_gpu.upload(merge_row1);
            // cv::cuda::resize(merge_row1_gpu, merge_row1_resize_gpu, cv::Size((int)(merge_row1.cols/4), (int)(merge_row1.rows/4)), 2); 
            // merge_row1_resize_gpu.download(merge_row1_resize);
            cv::resize(merge_row1, merge_row1_resize, cv::Size((int)(merge_row1.cols/4), (int)(merge_row1.rows/4)), 2);
            if (merge_row1_resize.empty()) 
            {
                std::cerr << "Failed to load image" << std::endl;
            }
            else
            {

                cv::imshow("xian_spreader_images_show01:", merge_row1_resize);
                cv::waitKey(10);


                //cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+"_tl_.jpg", tl_image);
                //cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+"tr_.jpg", tr_image);
                //cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+"bl_.jpg", bl_image);
                //cv::imwrite("/root/code/xian_aqc_ws/xian_project_file/trt/results/"+timeStr+"br_.jpg", br_image);
                // std::cerr << "iiiiiiiiiiiiiiiiiiiiii" << std::endl;
            }
            

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            std::cout << "FPS: " << 1000.0 / timediff << std::endl;
        } 


        

        
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_spreader_images_show_node");
    Xian_SpreaderImagesShow xian_spreader_images_show;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_spreader_images_show.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_SpreaderImagesShow::m_timer_HeartBeat_f, &xian_spreader_images_show);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}
