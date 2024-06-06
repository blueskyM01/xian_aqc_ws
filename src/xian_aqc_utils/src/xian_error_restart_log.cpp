#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "zpmc_cv_control.h"


class zpmc_ErrorRestartLog
{
    public:
        zpmc_ErrorRestartLog()
        {
            std::cout << "zpmc_error_restart_log:  节点已启动" << timeStr << std::endl;
            // 创建一个ROS节点句柄

            command = "mkdir -p " + log_path_dir;
            system(command.c_str());

            zpmc_log_file_name = log_path_dir + "error_restart.log";
            logFile.CreateFile(zpmc_log_file_name);

            // while (ros::ok())
            // {
            //     command_callback();
            //     usleep(1000 * 900);
            // }

        }

        ros::WallTimer m_timer_Main_Func;
        ros::WallTimer m_timer_HeartBeat;

        void m_timer_Main_Func_f(const ros::WallTimerEvent& event)
        {
            this->command_callback();
        }
        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_error_restart_log_heart_beat", xian_error_restart_log_heart_beat);
            std::cout << "xian_error_restart_log_heart_beat: " << xian_error_restart_log_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_error_restart_log_heart_beat",  counter);
        }

    private:

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
 
        std::string timeStr;
        std::string command;
        std::string command_kill_all = "ps -x | grep xian_ | grep -v 'grep' | kill `awk '{ print $1}'` | ps -x | grep noetic | grep -v 'grep' | kill `awk '{ print $1}'`";
        std::string command_kill_current_node = "";
        std::string command_restart_current_node = "";
        std::string log_path_dir = "/root/code/error_restart_log/";
        std::string zpmc_log_file_name;
        zpmc::SimpleLog logFile;
        std::string logline;

        // 手动重启
        // int zpmc_system_restart_flag = 0;
        // int zpmc_camera_node_restart_flag = 0;
        // int zpmc_accs_communicate_node_restart_flag = 0;

        // 心跳
        int xian_plc_heart_beat = 0; // 心跳频率1s
        int xian_camera_driver_heat_beat = 0; // 心跳频率1s
        int xian_container_corner_cop_process_heart_beat = 0; // 心跳频率1s
        int xian_keypoints_recognition_heart_beat = 0; // 心跳频率1s
        int xian_get_cell_guide_roi_heart_beat = 0; // 心跳频率1s
        int xian_cell_guide_recognition_heat_beat = 0; // 心跳频率1s
        int xian_cell_guide_mask_resize_heart_beat = 0; // 心跳频率1s
        int xian_cell_guide_point_identification_heart_beat = 0; // 心跳频率1s



        int xian_plc_heart_beat_cur = 0;
        int xian_plc_heart_beat_pre = 0;
        int xian_camera_driver_heat_beat_cur = 0;
        int xian_camera_driver_heat_beat_pre = 0;
        int xian_container_corner_cop_process_heart_beat_cur = 0;
        int xian_container_corner_cop_process_heart_beat_pre = 0;
        int xian_keypoints_recognition_heart_beat_cur = 0;
        int xian_keypoints_recognition_heart_beat_pre = 0;
        int xian_get_cell_guide_roi_heart_beat_cur = 0;
        int xian_get_cell_guide_roi_heart_beat_pre = 0;
        int xian_cell_guide_recognition_heat_beat_cur = 0;
        int xian_cell_guide_recognition_heat_beat_pre = 0;
        int xian_cell_guide_mask_resize_heart_beat_cur = 0;
        int xian_cell_guide_mask_resize_heart_beat_pre = 0;
        int xian_cell_guide_point_identification_heart_beat_cur = 0;
        int xian_cell_guide_point_identification_heart_beat_pre = 0;


        int xian_communication_with_plc_node_restart_flag = 0;
        int xian_camera_driver_restart_flag = 0;
        int xian_image_cop_process_restart_flag = 0; 
        int xian_keypoints_recognition_restart_flag = 0; 
        int xian_get_cell_guide_roi_restart_flag = 0; 
        int xian_cell_guide_recognition_restart_flag = 0; 
        int xian_cell_guide_mask_resize_restart_flag = 0; 
        int xian_cell_guide_point_identification_restart_flag = 0; 

        
        int xian_error_restart_log_heart_beat = 0;
        double xian_error_restart_log_fps = 0.0;

        int v_global_restart_max_count = 15;
        useconds_t v_restart_sleep_time = 1000 * 1000;

        void command_callback()
        {

            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            timeStr = zpmc::zpmc_get_stystem_time();
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = std::max(elapsedTimeP.count(), (long)(1));

            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_plc_heart_beat", xian_plc_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_camera_driver_heat_beat", xian_camera_driver_heat_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_container_corner_cop_process_heart_beat", xian_container_corner_cop_process_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_keypoints_recognition_heart_beat", xian_keypoints_recognition_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_get_cell_guide_roi_heart_beat", xian_get_cell_guide_roi_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_recognition_heat_beat", xian_cell_guide_recognition_heat_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_mask_resize_heart_beat", xian_cell_guide_mask_resize_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_point_identification_heart_beat", xian_cell_guide_point_identification_heart_beat);

            
            // xian_plc_heart_beat_pre = xian_plc_heart_beat_cur;
            // xian_plc_heart_beat_cur = xian_plc_heart_beat;
            // if(xian_plc_heart_beat_pre == xian_plc_heart_beat_cur)
            // {
            //     xian_communication_with_plc_node_restart_flag ++;
            // }
            // else
            // {
            //     xian_communication_with_plc_node_restart_flag = 0;
            // }
            // if(xian_communication_with_plc_node_restart_flag > v_global_restart_max_count)
            // {
            //     // 写入故障日志
            //     logline = timeStr + "   xian_communication_with_plc_node error";
            //     logFile.Write(logline);
            //     std::cout << logline << std::endl;
            //     // 重启节点
            //     command_kill_current_node = "ps -x | grep xian_communication_with_plc_node | grep -v 'grep' | kill `awk '{ print $1}'`";
            //     system(command_kill_current_node.c_str());
            //     usleep(v_restart_sleep_time);
            //     command_restart_current_node = "rosrun xian_communication_pkg xian_communication_with_plc_node &";
            //     system(command_restart_current_node.c_str());

            //     xian_communication_with_plc_node_restart_flag = 0; // 重启后多等20s
            // }

            // xian_camera_driver_heat_beat_pre = xian_camera_driver_heat_beat_cur;
            // xian_camera_driver_heat_beat_cur = xian_camera_driver_heat_beat;
            // if(xian_camera_driver_heat_beat_pre == xian_camera_driver_heat_beat_cur)
            // {
            //     xian_camera_driver_restart_flag ++;
            // }
            // else
            // {
            //     xian_camera_driver_restart_flag = 0;
            // }
            // if(xian_camera_driver_restart_flag > v_global_restart_max_count)
            // {
            //     // 写入故障日志
            //     logline = timeStr + "   xian_camera_driver node error";
            //     logFile.Write(logline);
            //     std::cout << logline << std::endl;
            //     // 重启节点
            //     command_kill_current_node = "ps -x | grep xian_camera_driver | grep -v 'grep' | kill `awk '{ print $1}'`";
            //     system(command_kill_current_node.c_str());
            //     usleep(v_restart_sleep_time);
            //     command_restart_current_node = "rosrun xian_sensor_pkg xian_camera_driver &";
            //     system(command_restart_current_node.c_str());
            //     xian_camera_driver_restart_flag = 0;
            // }

            xian_container_corner_cop_process_heart_beat_pre = xian_container_corner_cop_process_heart_beat_cur;
            xian_container_corner_cop_process_heart_beat_cur = xian_container_corner_cop_process_heart_beat;
            if(xian_container_corner_cop_process_heart_beat_pre == xian_container_corner_cop_process_heart_beat_cur)
            {
                xian_image_cop_process_restart_flag ++;
            }
            else
            {
                xian_image_cop_process_restart_flag = 0;
            }
            if(xian_image_cop_process_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                logline = timeStr + "   xian_image_cop_process node error";
                logFile.Write(logline);
                std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_image_cop_process | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_image_process xian_image_cop_process &";
                system(command_restart_current_node.c_str());

                xian_image_cop_process_restart_flag = 0;
            }

            xian_keypoints_recognition_heart_beat_pre = xian_keypoints_recognition_heart_beat_cur;
            xian_keypoints_recognition_heart_beat_cur = xian_keypoints_recognition_heart_beat;
            if(xian_keypoints_recognition_heart_beat_pre == xian_keypoints_recognition_heart_beat_cur)
            {
                xian_keypoints_recognition_restart_flag ++;
            }
            else
            {
                xian_keypoints_recognition_restart_flag = 0;
            }
            if(xian_keypoints_recognition_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                logline = timeStr + "   xian_keypoints_recognition node error";
                logFile.Write(logline);
                std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_keypoints_recognition | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_ai_pkg xian_keypoints_recognition.py &";
                system(command_restart_current_node.c_str());
                xian_keypoints_recognition_restart_flag = 0;
            }

            xian_get_cell_guide_roi_heart_beat_pre = xian_get_cell_guide_roi_heart_beat_cur;
            xian_get_cell_guide_roi_heart_beat_cur = xian_get_cell_guide_roi_heart_beat;
            if(xian_get_cell_guide_roi_heart_beat_pre == xian_get_cell_guide_roi_heart_beat_cur)
            {
                xian_get_cell_guide_roi_restart_flag ++;
            }
            else
            {
                xian_get_cell_guide_roi_restart_flag = 0;
            }
            if(xian_get_cell_guide_roi_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                logline = timeStr + "   xian_get_cell_guide_roi node error";
                logFile.Write(logline);
                std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_get_cell_guide_roi | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_image_process xian_get_cell_guide_roi &";
                system(command_restart_current_node.c_str());
                xian_get_cell_guide_roi_restart_flag = 0;
            }

            xian_cell_guide_recognition_heat_beat_pre = xian_cell_guide_recognition_heat_beat_cur;
            xian_cell_guide_recognition_heat_beat_cur = xian_cell_guide_recognition_heat_beat;
            if(xian_cell_guide_recognition_heat_beat_pre == xian_cell_guide_recognition_heat_beat_cur)
            {
                xian_cell_guide_recognition_restart_flag ++;
            }
            else
            {
                xian_cell_guide_recognition_restart_flag = 0;
            }
            if(xian_cell_guide_recognition_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                logline = timeStr + "   xian_cell_guide_recognition node error";
                logFile.Write(logline);
                std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_cell_guide_recognition | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_ai_pkg xian_cell_guide_recognition.py &";
                system(command_restart_current_node.c_str());
                xian_cell_guide_recognition_restart_flag = 0;
            }

            xian_cell_guide_mask_resize_heart_beat_pre = xian_cell_guide_mask_resize_heart_beat_cur;
            xian_cell_guide_mask_resize_heart_beat_cur = xian_cell_guide_mask_resize_heart_beat;
            if(xian_cell_guide_mask_resize_heart_beat_pre == xian_cell_guide_mask_resize_heart_beat_cur)
            {
                xian_cell_guide_mask_resize_restart_flag ++;
            }
            else
            {
                xian_cell_guide_mask_resize_restart_flag = 0;
            }
            if(xian_cell_guide_mask_resize_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                logline = timeStr + "   xian_cell_guide_mask_resize node error";
                logFile.Write(logline);
                std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_cell_guide_mask_resize | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_image_process xian_cell_guide_mask_resize &";
                system(command_restart_current_node.c_str());

                xian_cell_guide_mask_resize_restart_flag = 0;
            }

            xian_cell_guide_point_identification_heart_beat_pre = xian_cell_guide_point_identification_heart_beat_cur;
            xian_cell_guide_point_identification_heart_beat_cur = xian_cell_guide_point_identification_heart_beat;
            if(xian_cell_guide_point_identification_heart_beat_pre == xian_cell_guide_point_identification_heart_beat_cur)
            {
                xian_cell_guide_point_identification_restart_flag ++;
            }
            else
            {
                xian_cell_guide_point_identification_restart_flag = 0;
            }
            if(xian_cell_guide_point_identification_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                logline = timeStr + "   xian_cell_guide_point_identification node error";
                logFile.Write(logline);
                std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_cell_guide_point_identification | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_image_process xian_cell_guide_point_identification &";
                system(command_restart_current_node.c_str());

                xian_cell_guide_point_identification_restart_flag = 0;
            }
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_error_restart_log_fps",    1000.0 / timediff);

        } 
            
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"zpmc_error_restart_log");
    zpmc_ErrorRestartLog zpmc_error_restart_log;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    zpmc_error_restart_log.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(1.0), &zpmc_ErrorRestartLog::m_timer_Main_Func_f, &zpmc_error_restart_log);
    zpmc_error_restart_log.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &zpmc_ErrorRestartLog::m_timer_HeartBeat_f, &zpmc_error_restart_log);
    
    ros::waitForShutdown();
    // ros::spin();
    return 0;
}