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


class xian_CommunicationWithPlcSimulation
{
    public:
        xian_CommunicationWithPlcSimulation()
        {
            std::cout << "xian_communication_with_plc_simulation:  节点已启动" << timeStr << std::endl;

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
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_communication_with_plc_simulation_heart_beat", xian_communication_with_plc_simulation_heart_beat); // 自行替换
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_communication_with_plc_simulation_heart_beat: " << xian_communication_with_plc_simulation_heart_beat << std::endl;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_communication_with_plc_simulation_heart_beat", counter);  // 自行替换
        }

    private:

        

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
 
        std::string timeStr;

        int xian_cell_guide_detect_enable = 0;

        int pre_cell_guide_detect_enable = 0;
        int cur_cell_guide_detect_enable = 0;

        int xian_communication_with_plc_simulation_heart_beat = 0;
        double xian_communication_with_plc_simulation_fps = 0.0;
        
        int xian_plc_error_clear=0;
        int xian_plc_error_clear_flag=0;

        void command_callback()
        {

            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            timeStr = zpmc::zpmc_get_stystem_time();
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = std::max(elapsedTimeP.count(), (long)(1));
            
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_communication_with_plc_simulation_fps", xian_communication_with_plc_simulation_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_detect_enable", xian_cell_guide_detect_enable);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_plc_error_clear", xian_plc_error_clear);

            // 存储日志使能
            pre_cell_guide_detect_enable = cur_cell_guide_detect_enable;
            cur_cell_guide_detect_enable = xian_cell_guide_detect_enable;
            if(0 == pre_cell_guide_detect_enable && 1 == cur_cell_guide_detect_enable)
            {
                std::cout << "cur_cell_guide_detect_enable detected!" << std::endl;
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_cell_guide_detect_enable_simulation",                 1);
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_cell_guide_detect_enable_data_saving_simulation",     1);
            }
            
       	    if(1 == xian_plc_error_clear)
            {
		            xian_plc_error_clear_flag += 1;
		            if(xian_plc_error_clear_flag == 20)
		            {
		                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_plc_error_clear",                 0);
		                xian_plc_error_clear_flag = 0;
		            }
            }
            
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_communication_with_plc_simulation_fps",           1000.0 / timediff);
        }   

        
        int timer_counter = 0;
        int timer_counter_hb = 0;
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_communication_with_plc_simulation");
    xian_CommunicationWithPlcSimulation xian_communication_with_plc_simulation;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();    

    xian_communication_with_plc_simulation.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.05), &xian_CommunicationWithPlcSimulation::m_timer_Main_Func_f, &xian_communication_with_plc_simulation);
    xian_communication_with_plc_simulation.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &xian_CommunicationWithPlcSimulation::m_timer_HeartBeat_f, &xian_communication_with_plc_simulation);

    ros::waitForShutdown();
    // ros::spin();
    return 0;
}