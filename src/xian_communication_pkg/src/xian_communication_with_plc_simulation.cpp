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


class zpmc_CommunicationWithPlcSimulation
{
    public:
        zpmc_CommunicationWithPlcSimulation()
        {
            std::cout << "zpmc_communication_with_plc_simulation:  节点已启动" << timeStr << std::endl;

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
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_communication_with_plc_simulation_heart_beat", zpmc_communication_with_plc_simulation_heart_beat); // 自行替换
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "counter standalone: " << counter << std::endl;
            ros::param::set("/zpmc_unloading_parameters_node/zpmc_communication_with_plc_simulation_heart_beat", counter);  // 自行替换
        }

    private:

        

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
 
        std::string timeStr;

        int zpmc_unloading_enable = 0;
        int zpmc_unloading_enable_simulation = 0;
        int zpmc_loading_enable = 0;
        int zpmc_loading_enable_simulation = 0;
        int zpmc_firstlanding_enable = 0;
        int zpmc_firstlanding_enable_simulation = 0;

        int zpmc_is_parameters_save_successful_simulation = 0;
        int zpmc_is_parameters_save_successful = 0;

        int pre_unloading_enable = 0;
        int cur_unloading_enable = 0;
        int pre_loading_enable = 0;
        int cur_loading_enable = 0;
        int pre_firstlanding_enable = 0;
        int cur_firstlanding_enable = 0;

        int pre_zpmc_is_parameters_save_successful_simulation = 0;
        int cur_zpmc_is_parameters_save_successful_simulation = 0;

        int zpmc_communication_with_plc_simulation_heart_beat = 0;
        double zpmc_communication_with_plc_simulation_fps = 0.0;

        void command_callback()
        {

            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            timeStr = zpmc::zpmc_get_stystem_time();
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = std::max(elapsedTimeP.count(), (long)(1));
            // counter = counter > 1000 ? 0 : (counter + 1);

            // ros::param::get("/zpmc_unloading_parameters_node/zpmc_communication_with_plc_simulation_heart_beat", zpmc_communication_with_plc_simulation_heart_beat);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_communication_with_plc_simulation_fps", zpmc_communication_with_plc_simulation_fps);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_is_parameters_save_successful_simulation", zpmc_is_parameters_save_successful_simulation);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_is_parameters_save_successful", zpmc_is_parameters_save_successful);

            ros::param::get("/zpmc_unloading_parameters_node/zpmc_unloading_enable", zpmc_unloading_enable);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_unloading_enable_simulation", zpmc_unloading_enable_simulation);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_loading_enable_data_saving_simulation", zpmc_loading_enable_simulation);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_loading_enable", zpmc_loading_enable);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_firstlanding_enable_data_saving_simulation", zpmc_firstlanding_enable_simulation);
            ros::param::get("/zpmc_unloading_parameters_node/zpmc_firstlanding_enable", zpmc_firstlanding_enable);


            // 存储日志使能
            pre_unloading_enable = cur_unloading_enable;
            cur_unloading_enable = zpmc_unloading_enable;
            if(0 == pre_unloading_enable && 1 == cur_unloading_enable)
            {
                std::cout << "存储日志使能" << std::endl;
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_unloading_enable_simulation",                 1);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_unloading_enable_data_saving_simulation",     1);
            }
            pre_loading_enable = cur_loading_enable;
            cur_loading_enable = zpmc_loading_enable;
            if(0 == pre_loading_enable && 1 == cur_loading_enable)
            {
                std::cout << "存储日志使能" << std::endl;
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_loading_enable_simulation",                 1);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_loading_enable_data_saving_simulation",     1);
            }
            pre_firstlanding_enable = cur_firstlanding_enable;
            cur_firstlanding_enable = zpmc_firstlanding_enable;
            if(0 == pre_firstlanding_enable && 1 == cur_firstlanding_enable)
            {
                std::cout << "存储日志使能" << std::endl;
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_firstlanding_enable_simulation",                 1);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_firstlanding_enable_data_saving_simulation",     1);
            }

            

            // 参数是否保存成功
            pre_zpmc_is_parameters_save_successful_simulation = cur_zpmc_is_parameters_save_successful_simulation;
            cur_zpmc_is_parameters_save_successful_simulation = zpmc_is_parameters_save_successful_simulation;
            if(0 == pre_zpmc_is_parameters_save_successful_simulation && 1 == cur_zpmc_is_parameters_save_successful_simulation)
            {
                std::cout << "参数保存成功" << std::endl;
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_is_parameters_save_successful",               1);
            }

            std::cout
            << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            << std::endl
            << "zpmc_communication_with_plc_simulation:"                    << timeStr
            << std::endl
            << "zpmc_communication_with_plc_simulation_heart_beat:"         << zpmc_communication_with_plc_simulation_heart_beat 
            << "    zpmc_communication_with_plc_simulation_fps:"            << zpmc_communication_with_plc_simulation_fps
            << std::endl;

            // ros::param::set("/zpmc_unloading_parameters_node/zpmc_communication_with_plc_simulation_heart_beat",    counter);
            ros::param::set("/zpmc_unloading_parameters_node/zpmc_communication_with_plc_simulation_fps",           1000.0 / timediff);
        }   

        
        int timer_counter = 0;
        int timer_counter_hb = 0;
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"zpmc_communication_with_plc_simulation");
    zpmc_CommunicationWithPlcSimulation zpmc_communication_with_plc_simulation;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();    

    zpmc_communication_with_plc_simulation.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.05), &zpmc_CommunicationWithPlcSimulation::m_timer_Main_Func_f, &zpmc_communication_with_plc_simulation);
    zpmc_communication_with_plc_simulation.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &zpmc_CommunicationWithPlcSimulation::m_timer_HeartBeat_f, &zpmc_communication_with_plc_simulation);

    ros::waitForShutdown();
    // ros::spin();
    return 0;
}