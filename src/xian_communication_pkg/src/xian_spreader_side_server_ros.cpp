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

class Xian_SpreaderSideServerRos
{
    public:
        Xian_SpreaderSideServerRos()
        {
            std::cout << "xian_spreader_side_server_ros_node:  节点已启动" << timeStr << std::endl;
            init();
        }

        ~Xian_SpreaderSideServerRos()
        {
            close(server_fd);
        }

        

        ros::WallTimer m_timer_Main_Func;
        ros::WallTimer m_timer_HeartBeat;


        void m_timer_Main_Func_f(const ros::WallTimerEvent& event)
        {
            this->command_callback();
        }
        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_spreader_side_server_ros_heart_beat", xian_spreader_side_server_ros_heart_beat); 
            std::cout << "xian_spreader_side_server_ros_heart_beat: " << xian_spreader_side_server_ros_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_spreader_side_server_ros_heart_beat", counter);  
        }

    private:

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0; 
        std::string timeStr;

        int server_fd, new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);
        std::string ip = "10.28.137.12";
        char const* trolley_client_ip = ip.c_str();

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
        
        int xian_retrable_box_state0 = 0;
        int xian_retrable_box_state1 = 0;
        int xian_retrable_box_state2 = 0;
        int xian_retrable_box_state3 = 0;
        int xian_from_plc_to_retrable_box_mode0 = 0;
        int xian_from_plc_to_retrable_box_mode1 = 0;
        int xian_from_plc_to_retrable_box_mode2 = 0;
        int xian_from_plc_to_retrable_box_mode3 = 0;
        int xian_spreader_side_server_ros_heart_beat = 0;
        int xian_plc_heart_beat = 0;

        

        void init()
        {
            // Create socket
            if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
                perror("socket failed");
                exit(EXIT_FAILURE);
            }

            // Set socket options
            if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
                perror("setsockopt");
                exit(EXIT_FAILURE);
            }

            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            // address.sin_addr.s_addr = inet_addr(trolley_client_ip);
            
            address.sin_port = htons(PORT);

            // Bind socket to port
            if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
                perror("bind failed");
                exit(EXIT_FAILURE);
            }

            // Start listening
            if (listen(server_fd, 3) < 0) {
                perror("listen");
                exit(EXIT_FAILURE);
            }

            printf("Server listening on port %d...\n", PORT);
        }

        void command_callback()
        {                           

            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) 
            {
                perror("accept");
                // continue;
            }
            else
            {
                printf("Client connected\n");
                handle_client(new_socket);
            }
            
            
        }




        void handle_client(int client_socket) 
        {
            struct trolley2spreader trolley_data;
            struct spreader2trolley spreader_data;
            int counter = 0;

            

            // Communication loop
            while (1) 
            {
                usleep(50 * 1000); // 50 ms
                // Prepare data to send
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state0", xian_retrable_box_state0);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state1", xian_retrable_box_state1);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state2", xian_retrable_box_state2);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state3", xian_retrable_box_state3);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode0", xian_from_plc_to_retrable_box_mode0);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode1", xian_from_plc_to_retrable_box_mode1);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode2", xian_from_plc_to_retrable_box_mode2);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_from_plc_to_retrable_box_mode3", xian_from_plc_to_retrable_box_mode3);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_plc_heart_beat", xian_plc_heart_beat); 
                spreader_data.State0 = xian_retrable_box_state0;
                spreader_data.State1 = xian_retrable_box_state1;
                spreader_data.State2 = xian_retrable_box_state2;
                spreader_data.State3 = xian_retrable_box_state3;
                spreader_data.mode0 = xian_from_plc_to_retrable_box_mode0;
                spreader_data.mode1 = xian_from_plc_to_retrable_box_mode1;
                spreader_data.mode2 = xian_from_plc_to_retrable_box_mode2;
                spreader_data.mode3 = xian_from_plc_to_retrable_box_mode3;
                spreader_data.spreader_heart_beat = xian_plc_heart_beat;

                // Send data to client
                if (send(client_socket, &spreader_data, sizeof(spreader2trolley), 0) < 0) {
                    printf("Send failed\n");
                    close(client_socket);
                    return;
                }
                printf("Data sent to client\n");

                // Receive data from client
                if (recv(client_socket, &trolley_data, sizeof(trolley2spreader), 0) <= 0) {
                    printf("Client disconnected\n");
                    close(client_socket);
                    return;
                }
                printf("Spreader Side Received: mode0=%d, mode1=%d, mode2=%d, mode3=%d, trolley_heart_beat=%d\n",
                    trolley_data.mode0, trolley_data.mode1, trolley_data.mode2, trolley_data.mode3, trolley_data.trolley_heart_beat);
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", trolley_data.mode0);  
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", trolley_data.mode1);  
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", trolley_data.mode2);  
                ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", trolley_data.mode3);  
            }
        }

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_spreader_side_server_ros_node");
    Xian_SpreaderSideServerRos xian_spreader_side_server_ros_node;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_spreader_side_server_ros_node.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1), &Xian_SpreaderSideServerRos::m_timer_HeartBeat_f, &xian_spreader_side_server_ros_node);
    xian_spreader_side_server_ros_node.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.05), &Xian_SpreaderSideServerRos::m_timer_Main_Func_f, &xian_spreader_side_server_ros_node);

    ros::waitForShutdown();

    // ros::spin();
    return 0;
}