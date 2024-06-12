#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include<iostream>


class Xian_RetractableBoxWorkShow
{
    public:
        Xian_RetractableBoxWorkShow()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            
            for(int i=0; i < 10; i++)
            {
            
            
            std::cout << "Start 1" << std::endl;
            mode0 = 0;
            mode1 = 1;
            mode2 = 1;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 2" << std::endl;
            mode0 = 0;
            mode1 = 0;
            mode2 = 1;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 3" << std::endl;
            mode0 = 0;
            mode1 = 0;
            mode2 = 0;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 4" << std::endl;
            mode0 = 0;
            mode1 = 0;
            mode2 = 0;
            mode3 = 0;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 5" << std::endl;
            mode0 = 0;
            mode1 = 0;
            mode2 = 0;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 6" << std::endl;
            mode0 = 0;
            mode1 = 0;
            mode2 = 1;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 7" << std::endl;
            mode0 = 0;
            mode1 = 1;
            mode2 = 1;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 8" << std::endl;
            mode0 = 1;
            mode1 = 1;
            mode2 = 1;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 9" << std::endl;
            mode0 = 1;
            mode1 = 0;
            mode2 = 0;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 10" << std::endl;
            mode0 = 0;
            mode1 = 1;
            mode2 = 1;
            mode3 = 0;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            usleep(1000000 * time_minutes); //��ʱtime_minutes��
            
            std::cout << "Start 11" << std::endl;
            mode0 = 1;
            mode1 = 1;
            mode2 = 1;
            mode3 = 1;
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2); 
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3); 
            
            }

        }

        ros::WallTimer m_timer_HeartBeat;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "Xian_retractable_box_work_show_heart_beat: " << counter << std::endl;
        }

    private:
        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        std::string timeStr;     
        int mode0 = 0;
        int mode1 = 0;
        int mode2 = 0;
        int mode3 = 0; 
        double time_minutes = 1;     
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_retractable_box_work_show_node");
    Xian_RetractableBoxWorkShow Xian_retractable_box_work_show;

    // ����һ��ROS�ڵ���
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    Xian_retractable_box_work_show.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_RetractableBoxWorkShow::m_timer_HeartBeat_f, &Xian_retractable_box_work_show);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}