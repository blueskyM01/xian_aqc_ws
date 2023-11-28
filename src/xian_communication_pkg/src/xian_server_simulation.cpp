#include<ros/ros.h>
#include <ros/callback_queue.h>
#include "boost/thread.hpp"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include<arpa/inet.h>

#include<stdio.h>
#include<stdlib.h>
#include<string.h>



class Xian_ServerSimulation
{
public:
    Xian_ServerSimulation()
    {
        std::cout << "xian_server_simulation node has been started!" << std::endl;
        init();
    }

    
    ~Xian_ServerSimulation()
    {
        close(server_socket);
    }
    

    ros::WallTimer m_timer_Main_Func;
    ros::WallTimer m_timer_HeartBeat;

    void m_timer_Main_Func_f(const ros::WallTimerEvent& event)
    {
        this->command_callback();
    }
    void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
    {
        ros::param::get("/xian_aqc_dynamic_parameters_server/xian_server_simulation_node_heart_beat", xian_server_simulation_node_heart_beat); // 自行替换
        counter = counter > 1000 ? 0 : (counter + 1);
        std::cout << "counter standalone: " << xian_server_simulation_node_heart_beat << std::endl;
        ros::param::set("/xian_aqc_dynamic_parameters_server/xian_server_simulation_node_heart_beat", counter);  // 自行替换
    }

    void init()
    {
        // Create socket
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if(server_socket == -1) 
        {
            std::cout << "Socket creation failed!"<< std::endl;
            exit(1);
        }

        // Set up server address structure
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(PORT);
        server_address.sin_addr.s_addr = inet_addr("0.0.0.0");

        // Bind the socket
        if(bind(server_socket, reinterpret_cast<struct sockaddr*>(&server_address), sizeof(server_address)) == -1) 
        {
            std::cout << "Bind failed!" << std::endl;
            exit(-1);
        }

        // Listen for incoming connections
        if(listen(server_socket, 5) == -1) 
        {
            std::cout << "Listen failed!" << std::endl;
            exit(-1);
        }

        std::cout << "Server listening on port " << PORT << std::endl;

        // // Accept and handle incoming connections
        // while (true) 
        // {
        //     client_socket = accept(server_socket, reinterpret_cast<struct sockaddr*>(&client_address), &client_address_len);
        //     if (client_socket == -1) 
        //     {
        //         std::cout << "Accept failed!" << std::endl;
        //         continue;
        //     }

        //     std::cout << "Connection accepted from " << inet_ntoa(client_address.sin_addr) << ":" << ntohs(client_address.sin_port) << std::endl;

        //     // Handle the client in a separate thread or function
        //     handle_client(client_socket);
        // }

        // // Close the server socket
        // close(server_socket);

    }

    void handle_client(int client_socket) 
    {
        // char buffer[BUFFER_SIZE];
        ssize_t bytes_received;
        int tcp_server_heart_beat = 0;
        while (true) 
        {
            usleep(30*1000);
            if(tcp_server_heart_beat > 1000)
            {
                tcp_server_heart_beat = 0;
            }
            tcp_server_heart_beat ++;
            // Receive data from the client
            // bytes_received = recv(client_socket, buffer, sizeof(buffer), 0);

            RetractableBoxToServer read_from_retractable_box;
            bytes_received = read(client_socket, &read_from_retractable_box, sizeof(read_from_retractable_box));
            if (bytes_received <= 0) {
                // Connection closed or error, break the loop
                break;
            }
            
            int tcp_retrable_box_heart_beat_tl = read_from_retractable_box.tcp_retrable_box_heart_beat_tl;
            int tcp_retrable_box_heart_beat_tr = read_from_retractable_box.tcp_retrable_box_heart_beat_tr;
            int tcp_retrable_box_heart_beat_bl = read_from_retractable_box.tcp_retrable_box_heart_beat_bl;
            int tcp_retrable_box_heart_beat_br = read_from_retractable_box.tcp_retrable_box_heart_beat_br;
            int retractable_motion_flag_tl = read_from_retractable_box.retractable_motion_flag_tl;
            int retractable_motion_flag_tr = read_from_retractable_box.retractable_motion_flag_tr;
            int retractable_motion_flag_bl = read_from_retractable_box.retractable_motion_flag_bl;
            int retractable_motion_flag_br = read_from_retractable_box.retractable_motion_flag_br;
            int retractable_box_status_tl = read_from_retractable_box.retractable_box_status_tl;
            int retractable_box_status_tr = read_from_retractable_box.retractable_box_status_tr;
            int retractable_box_status_bl = read_from_retractable_box.retractable_box_status_bl;
            int retractable_box_status_br = read_from_retractable_box.retractable_box_status_br;
            int error_code_tl = read_from_retractable_box.error_code_tl;
            int error_code_tr = read_from_retractable_box.error_code_tr;
            int error_code_bl = read_from_retractable_box.error_code_bl;
            int error_code_br = read_from_retractable_box.error_code_br;


            int iWriteCount = 0;
            ServerToRetractableBox send_to_retractable_box;
            send_to_retractable_box.tcp_server_heart_beat = tcp_server_heart_beat;
            ros::param::get("/xian_aqc_dynamic_parameters_server/auto_manual_switch_flag", auto_manual_switch_flag); // 自行替换
            send_to_retractable_box.auto_manual_switch_flag = auto_manual_switch_flag;

            iWriteCount = write(client_socket, &send_to_retractable_box, sizeof(send_to_retractable_box));    

            if(tcp_server_heart_beat % 30 == 0)
            {
                printf("tcp_retrable_box_heart_beat_tl: %d \n", tcp_retrable_box_heart_beat_tl);
                printf("tcp_retrable_box_heart_beat_tr: %d \n", tcp_retrable_box_heart_beat_tr);
                printf("tcp_retrable_box_heart_beat_bl: %d \n", tcp_retrable_box_heart_beat_bl);
                printf("tcp_retrable_box_heart_beat_br: %d \n", tcp_retrable_box_heart_beat_br);
                printf("retractable_motion_flag_tl: %d \n", retractable_motion_flag_tl);
                printf("retractable_motion_flag_tr: %d \n", retractable_motion_flag_tr);
                printf("retractable_motion_flag_bl: %d \n", retractable_motion_flag_bl);
                printf("retractable_motion_flag_br: %d \n", retractable_motion_flag_br);
                printf("retractable_box_status_tl: %d \n", retractable_box_status_tl);
                printf("retractable_box_status_tr: %d \n", retractable_box_status_tr);
                printf("retractable_box_status_bl: %d \n", retractable_box_status_bl);
                printf("retractable_box_status_br: %d \n", retractable_box_status_br);
                printf("error_code_tl: %d \n", error_code_tl);
                printf("error_code_tr: %d \n", error_code_tr);
                printf("error_code_bl: %d \n", error_code_bl);
                printf("error_code_br: %d \n", error_code_br);
                printf("tcp_server_heart_beat: %d \n", send_to_retractable_box.tcp_server_heart_beat);
                printf("auto_manual_switch_flag: %d \n", send_to_retractable_box.auto_manual_switch_flag);
                printf("--------------------------------------------------------------------");
            }


            
        }

        // Close the socket when the client disconnects
        close(client_socket);
    }

    void command_callback()
    {
        client_socket = accept(server_socket, reinterpret_cast<struct sockaddr*>(&client_address), &client_address_len);
        if(client_socket == -1) 
        {
            std::cout << "Accept failed!" << std::endl;
            // continue;
        }
        else
        {
            std::cout << "Connection accepted from " << inet_ntoa(client_address.sin_addr) << ":" << ntohs(client_address.sin_port) << std::endl;

            // Handle the client in a separate thread or function
            handle_client(client_socket);
        }
    }

private:
    int counter = 0;
    int xian_server_simulation_node_heart_beat = 0;
    int auto_manual_switch_flag = 0;

    int server_socket, client_socket;
    sockaddr_in server_address, client_address;
    socklen_t client_address_len = sizeof(client_address);
    int PORT = 2820;
    // int BUFFER_SIZE = 1024;

    struct RetractableBoxToServer
    {
        int tcp_retrable_box_heart_beat_tl = 0;    // tcp client heart beat
        int tcp_retrable_box_heart_beat_tr = 0;
        int tcp_retrable_box_heart_beat_bl = 0;
        int tcp_retrable_box_heart_beat_br = 0;
        int retractable_motion_flag_tl = 0;  // 0: stop 1: in processing
        int retractable_motion_flag_tr = 0;
        int retractable_motion_flag_bl = 0;
        int retractable_motion_flag_br = 0;
        int retractable_box_status_tl = 0;   // 0: extent, 1: retract
        int retractable_box_status_tr = 0;
        int retractable_box_status_bl = 0;
        int retractable_box_status_br = 0;
        int error_code_tl = 9000;               // 9001: 
        int error_code_tr = 9000;
        int error_code_bl = 9000;
        int error_code_br = 9000;
    };

    struct ServerToRetractableBox
    {
        int tcp_server_heart_beat = 0;       // heart beat
        int auto_manual_switch_flag = 0;     // 0: auto; 1:manual
    };

        
};

int main(int argc, char** argv) 
{

    //initial and name node
    ros::init(argc,argv,"xian_server_simulation");
    Xian_ServerSimulation xian_server_simulation_node;
    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_server_simulation_node.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1), &Xian_ServerSimulation::m_timer_HeartBeat_f, &xian_server_simulation_node);
    xian_server_simulation_node.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.05), &Xian_ServerSimulation::m_timer_Main_Func_f, &xian_server_simulation_node);

    ros::waitForShutdown();
    return 0;
}
