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
        char buffer[BUFFER_SIZE];
        ssize_t bytes_received;
        int tcp_server_heart_beat = 0;
        while (true) 
        {
            usleep(50*1000);
            // Receive data from the client
            // bytes_received = recv(client_socket, buffer, sizeof(buffer), 0);
            RetractableBox retract_pkg_read_from_sensor_box;
            bytes_received = read(client_socket, &retract_pkg_read_from_sensor_box, sizeof(retract_pkg_read_from_sensor_box));

            if (bytes_received <= 0) {
                // Connection closed or error, break the loop
                break;
            }
            std::cout << "recevie from box:" << std::endl;
            std::cout << "retract_pkg_read_from_sensor_box.heart_beat: " << retract_pkg_read_from_sensor_box.heart_beat << std::endl;
            std::cout << "retract_pkg_read_from_sensor_box.auto_manual_switch_flag: " << retract_pkg_read_from_sensor_box.auto_manual_switch_flag << std::endl;
            std::cout << "retract_pkg_read_from_sensor_box.PLC_retractable_order: " << retract_pkg_read_from_sensor_box.PLC_retractable_order << std::endl;


            int iWriteCount = 0;
            RetractableBox retract_pkg_send_to_plc;
            retract_pkg_send_to_plc.heart_beat = tcp_server_heart_beat;
            retract_pkg_send_to_plc.auto_manual_switch_flag = 111;
            retract_pkg_send_to_plc.PLC_retractable_order = 112;
            iWriteCount = write(client_socket, &retract_pkg_send_to_plc, sizeof(retract_pkg_send_to_plc));    
            tcp_server_heart_beat ++;
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

    int server_socket, client_socket;
    sockaddr_in server_address, client_address;
    socklen_t client_address_len = sizeof(client_address);
    int PORT = 2820;
    int BUFFER_SIZE = 1024;

    struct RetractableBox
    {
        int heart_beat; // heart beat
        int auto_manual_switch_flag; // 0: auto; 1:manual
        int PLC_retractable_order;    // 0: extent, 1: retract
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
