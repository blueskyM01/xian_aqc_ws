#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include <ros/callback_queue.h>
#include "boost/thread.hpp"

#define PORT 2840

class Xian_TrolleySideClientRos
{
    public:
        Xian_TrolleySideClientRos()
        {
            std::cout << "xian_trolley_side_client_ros_node:  节点已启动" << timeStr << std::endl;
        }

        ~Xian_TrolleySideClientRos()
        {
            
        }

        ros::WallTimer m_timer_Main_Func;
        ros::WallTimer m_timer_HeartBeat;


        void m_timer_Main_Func_f(const ros::WallTimerEvent& event)
        {
            this->command_callback();
        }
        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_trolley_side_client_ros_heart_beat", xian_trolley_side_client_ros_heart_beat); 
            std::cout << "xian_trolley_side_client_ros_heart_beat: " << xian_trolley_side_client_ros_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_trolley_side_client_ros_heart_beat", counter);  
        }

    private:

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0; 
        std::string timeStr;

        int sock = 0;
        struct sockaddr_in serv_addr;
        struct trolley2spreader {
            int mode0; 
            int mode1; 
            int mode2;    
            int mode3; 
            int trolley_heart_beat;
        };

        struct spreader2trolley {
            int State0; 
            int State1; 
            int State2;    
            int State3; 
            int mode0; 
            int mode1; 
            int mode2;    
            int mode3; 
            int spreader_heart_beat;
        };
        
        int xian_acds_send_to_retrable_box_mode0 = 0;
        int xian_acds_send_to_retrable_box_mode1 = 0;
        int xian_acds_send_to_retrable_box_mode2 = 0;
        int xian_acds_send_to_retrable_box_mode3 = 0;
        int xian_trolley_side_client_ros_heart_beat = 0;

        void command_callback()
        {                           

            // Create socket
            if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                printf("\n Socket creation error \n");
                exit(-1);
            }

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(PORT);

            // Convert IPv4 and IPv6 addresses from text to binary form
            if (inet_pton(AF_INET, "10.28.137.11", &serv_addr.sin_addr) <= 0) {
                printf("\nInvalid address/ Address not supported \n");
                exit(-1);
            }

            // Try to connect to server
            while (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                printf("\nConnection Failed \n");
                close(sock);
                sleep(1); // Wait for 1 second before retrying
            }

            printf("Connected to server\n");
            handle_communication(sock);
            
        }







        void handle_communication(int sock) 
        {
            struct trolley2spreader trolley_data;
            struct spreader2trolley spreader_data;
            int counter = 0;

            // Communication loop
            while (1) {
                // Prepare data to send
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", xian_acds_send_to_retrable_box_mode0);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", xian_acds_send_to_retrable_box_mode1);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", xian_acds_send_to_retrable_box_mode2);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", xian_acds_send_to_retrable_box_mode3);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_trolley_side_client_ros_heart_beat", xian_trolley_side_client_ros_heart_beat); 
                trolley_data.mode0 = xian_acds_send_to_retrable_box_mode0;
                trolley_data.mode1 = xian_acds_send_to_retrable_box_mode1;
                trolley_data.mode2 = xian_acds_send_to_retrable_box_mode2;
                trolley_data.mode3 = xian_acds_send_to_retrable_box_mode3;
                trolley_data.trolley_heart_beat = xian_trolley_side_client_ros_heart_beat;
                // Send data to server
                if (send(sock, &trolley_data, sizeof(trolley2spreader), 0) < 0) {
                    printf("\nSend failed\n");
                    close(sock);
                    return;
                }
                printf("Data sent to server\n");

                // Receive data from server
                if (recv(sock, &spreader_data, sizeof(spreader2trolley), 0) <= 0) {
                    printf("\nServer disconnected\n");
                    close(sock);
                    return;
                }
                printf("Trolley Side Received: State0=%d, State1=%d, State2=%d,  State3=%d, mode0=%d, mode1=%d, mode2=%d, mode3=%d, spreader_heart_beat=%d\n",
                    spreader_data.State0, spreader_data.State1, spreader_data.State2, spreader_data.State3,
                    spreader_data.mode0, spreader_data.mode1, spreader_data.mode2, spreader_data.mode3,
                    spreader_data.spreader_heart_beat);
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_spreader_side_server_ros_heart_beat", spreader_data.spreader_heart_beat);  
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode0", spreader_data.mode0); 
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode1", spreader_data.mode1); 
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode2", spreader_data.mode2); 
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode3", spreader_data.mode3); 
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state0", spreader_data.State0);
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state1", spreader_data.State1);
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state2", spreader_data.State2);
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state3", spreader_data.State3);
                // Optionally, you can add a sleep here to control the rate of communication
                usleep(50 * 1000); // 50 ms
            }
        }

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_trolley_side_client_ros_node");
    Xian_TrolleySideClientRos xian_trolley_side_client_ros_node;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_trolley_side_client_ros_node.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1), &Xian_TrolleySideClientRos::m_timer_HeartBeat_f, &xian_trolley_side_client_ros_node);
    xian_trolley_side_client_ros_node.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.05), &Xian_TrolleySideClientRos::m_timer_Main_Func_f, &xian_trolley_side_client_ros_node);

    ros::waitForShutdown();

    // ros::spin();
    return 0;
}