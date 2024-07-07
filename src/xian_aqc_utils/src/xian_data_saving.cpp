#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>

#include "xian_msg_pkg/xian_spreader_images_msg.h"



#include "zpmc_cv_control.h"


class xian_DataSaving
{
    public:
        xian_DataSaving()
        {
            std::cout << "xian_data_saving:  节点已启动" << timeStr << std::endl;
            // 创建一个ROS节点句柄

            ros::NodeHandle nh;
            command_subscribe_ = nh.subscribe<xian_msg_pkg::xian_spreader_images_msg>("xian_aqc_spreader_images", 1, &xian_DataSaving::command_callback, this);
        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_data_saving_heart_beat", xian_data_saving_heart_beat); // 自行替换
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_data_saving_heart_beat: " << xian_data_saving_heart_beat << std::endl;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_data_saving_heart_beat", counter);  // 自行替换
        }

    private:

        ros::Subscriber command_subscribe_;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        std::string timeStr;

        std::string log_path_dir = "/root/code/log/origin_data_saving/";
        std::string command;
        // cv::Size size = cv::Size(1024, 1280);
        cv::Size size = cv::Size(1920, 1536);
        int myFourCC = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        cv::VideoWriter output_video;



        struct rec_img_struct
        {
            cv::Mat rec_img_tl;
            cv::Mat rec_img_tr;
            cv::Mat rec_img_bl;
            cv::Mat rec_img_br;
        };
        rec_img_struct rec_img;


        struct output_video_struct
        {
            cv::VideoWriter tl;
            cv::VideoWriter tr;
            cv::VideoWriter bl;
            cv::VideoWriter br;
        };
        struct output_video_type_struct
        {
            output_video_struct cell_guide_detect;
        };
        output_video_type_struct output_video_s;

        struct zpmc_log_video_name_struct
        {
            std::string tl;
            std::string tr;
            std::string bl;
            std::string br;
        };
        struct zpmc_log_video_name_type_struct
        {
            zpmc_log_video_name_struct cell_guide_detect;
        };
        zpmc_log_video_name_type_struct zpmc_log_video_name;

        // 原始数据保存标志位
        int xian_cell_guide_detect_enable_data_saving_simulation = 0;

        // 使能标志位
        int xian_cell_guide_detect_enable = 0;

        int xian_data_saving_heart_beat = 0;
        double xian_data_saving_fps = 0.0;


        void command_callback(const xian_msg_pkg::xian_spreader_images_msgConstPtr& zpmc_spreader_camera_images_msg)
        {

            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            timeStr = zpmc::zpmc_get_stystem_time();
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = std::max(elapsedTimeP.count(), (long)(1));
            
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_data_saving_fps", xian_data_saving_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_detect_enable_data_saving_simulation", xian_cell_guide_detect_enable_data_saving_simulation);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_detect_enable", xian_cell_guide_detect_enable);

            rec_img.rec_img_tl = cv_bridge::toCvShare(zpmc_spreader_camera_images_msg->tl_image, zpmc_spreader_camera_images_msg,"bgr8")->image;
            rec_img.rec_img_tr = cv_bridge::toCvShare(zpmc_spreader_camera_images_msg->tr_image, zpmc_spreader_camera_images_msg,"bgr8")->image;
            rec_img.rec_img_bl = cv_bridge::toCvShare(zpmc_spreader_camera_images_msg->bl_image, zpmc_spreader_camera_images_msg,"bgr8")->image;
            rec_img.rec_img_br = cv_bridge::toCvShare(zpmc_spreader_camera_images_msg->br_image, zpmc_spreader_camera_images_msg,"bgr8")->image;

            if(xian_cell_guide_detect_enable_data_saving_simulation == 1)
            {
                // 创建存储视频的文件夹
                std::string new_folder;
                if (xian_cell_guide_detect_enable_data_saving_simulation == 1)
                {
                    new_folder = log_path_dir + timeStr + "-cell_guide_detection/";
                }
                else
                {
                    new_folder = log_path_dir + timeStr + "-unknown/";
                }
                
                command = "mkdir -p " + new_folder;
                system(command.c_str());

                output_video_s.cell_guide_detect.tl.release();
                zpmc_log_video_name.cell_guide_detect.tl = new_folder + "cam1_000.avi";
                output_video_s.cell_guide_detect.tl.open(zpmc_log_video_name.cell_guide_detect.tl, myFourCC, 10, size, true);
                std::cout << "Generate log video_tl at " << zpmc_log_video_name.cell_guide_detect.tl << std::endl;

                output_video_s.cell_guide_detect.tr.release();
                zpmc_log_video_name.cell_guide_detect.tr = new_folder + "cam2_000.avi";
                output_video_s.cell_guide_detect.tr.open(zpmc_log_video_name.cell_guide_detect.tr, myFourCC, 10, size, true);
                std::cout << "Generate log video_tr at " << zpmc_log_video_name.cell_guide_detect.tr << std::endl;

                output_video_s.cell_guide_detect.bl.release();
                zpmc_log_video_name.cell_guide_detect.bl = new_folder + "cam3_000.avi";
                output_video_s.cell_guide_detect.bl.open(zpmc_log_video_name.cell_guide_detect.bl, myFourCC, 10, size, true);
                std::cout << "Generate log video_bl at " << zpmc_log_video_name.cell_guide_detect.bl << std::endl;

                output_video_s.cell_guide_detect.br.release();
                zpmc_log_video_name.cell_guide_detect.br = new_folder + "cam4_000.avi";
                output_video_s.cell_guide_detect.br.open(zpmc_log_video_name.cell_guide_detect.br, myFourCC, 10, size, true);
                std::cout << "Generate log video_br at " << zpmc_log_video_name.cell_guide_detect.br << std::endl;
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_cell_guide_detect_enable_data_saving_simulation", 0);
 
            }

            if(xian_cell_guide_detect_enable == 1)
            {
                // output_video_s << merge_log;
                output_video_s.cell_guide_detect.tl << rec_img.rec_img_tl;
                output_video_s.cell_guide_detect.tr << rec_img.rec_img_tr;
                output_video_s.cell_guide_detect.bl << rec_img.rec_img_bl;
                output_video_s.cell_guide_detect.br << rec_img.rec_img_br;
                printf("write source images! \n");
            }

            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_data_saving_fps", 1000.0 / timediff);


        } 
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_data_saving");
    xian_DataSaving xian_data_saving;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_data_saving.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &xian_DataSaving::m_timer_HeartBeat_f, &xian_data_saving);
    
    ros::waitForShutdown();
    // ros::spin();
    return 0;
}