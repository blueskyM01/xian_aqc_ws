#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>

#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>

#include <ros/callback_queue.h>
#include "boost/thread.hpp"


class zpmc_CommunicationWithAccs
{
    public:
        zpmc_CommunicationWithAccs()
        {
            std::cout << "xian_communication_with_plc_node:  节点已启动" << timeStr << std::endl;
            init();
        }

        ros::WallTimer m_timer_Main_Func;
        ros::WallTimer m_timer_HeartBeat;


        void m_timer_Main_Func_f(const ros::WallTimerEvent& event)
        {
            this->command_callback();
        }
        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_heart_beat", xian_acds_heart_beat); 
            std::cout << "xian_acds_heart_beat: " << xian_acds_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_heart_beat", counter);  
        }

    private:

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0; 
        std::string timeStr;


        // ------用来从获取ros系统中的参数--------
        bool is_accs_connect = false;

        // ------用来从获取ros系统中的参数--------


        // json zpmc_j20;
        // 从文件读取20feet配置
        // std::ifstream zpmc_20feet("/zpmc/project_file/09_param_dir/zpmc_20f.json");   
        // std::ifstream zpmc_20feet;
        std::string accs_ip, local_ip_str;
        int PORT = 2812;

        char const* server_ip;
        char const* local_ip;

        struct sockaddr_in addr;
        struct sockaddr_in addr_local;

        struct timeval tvTimeout;
        struct timeval tvTimeout_recv;
        int socket_fd = -1;
        int connect_flag;
        int fail_count_send = 0;
        int fail_count_recv = 0;
        int res, res_bind;
        // Socket m_socket;

        // -------从plc中读参数--------
        // unsigned char plc_buffer[78] = {};
        unsigned char plc_buffer[86 * 10] = {};
        struct PLC2AFLS
        {
            int header1=0;
            int header2=1;
            int header3=2;
            int header4=3;
            int send_len=4;
            int Heart_Beat_PLC=6;
            int Spreader_Size=8;
            int Spreader_Lock_Signal=10;
            int Spreader_Land_Signal=12;
            int Gantry_Position=14;
            int Trolley_Position=18;
            int Hoist_Position=22;
            int Target_Gantry_Position=26;
            int Target_Trolley_Position=30;
            int Accs_Job_ID=34;
            int Cmd_Index=38;
            int Cmd_Type=39;
            int Cmd_Add_Info=40;
            int Sea_Land_Side_single=41;
            int Gantry_Speed=42;
            int Trolley_Speed=44;
            int Hoist_Speed=46;
            int Target_Hoist_Position=48;
            int Capture_Image_Flag=50;
            int TDS_Left_Control_X=52;
            int TDS_Left_Control_Y=54;
            int TDS_Right_Control_X=56;
            int TDS_Right_Control_Y=58;
            int SDS_Left_Control_X=60;
            int SDS_Left_Control_Y=62;
            int SDS_Right_Control_X=64;
            int SDS_Right_Control_Y=66;
            int AFLS_Enable=68;
            int state0=70;
            int state1=72;
            int state2=74;
            int state3=76;
            int mode0=78;
            int mode1=80;
            int mode2=82;
            int mode3=84;
        };
        PLC2AFLS recv_plc;
        // -------从plc中读参数--------

        // -------向服务端发送的参数------
        struct AFLS2PLC
        {
            uint8_t header1;
            uint8_t header2;
            uint8_t header3;
            uint8_t header4;
            uint16_t send_len;
            uint8_t Cmd_Index_Fb;
            uint8_t Cmd_Type_Fb;
            int16_t Heart_Beat_AFLS;
            int16_t AFLS_Status;
            int16_t Camera_Status;
            uint16_t AFLS_Error_Code;
            int16_t Left_Top_Diff_X;
            int16_t Left_Top_Diff_Y;
            int16_t Right_Top_Diff_X;
            int16_t Right_Top_Diff_Y;
            int16_t Left_Low_Diff_X;
            int16_t Left_Low_Diff_Y;
            int16_t Right_Low_Diff_X;
            int16_t Right_Low_Diff_Y;
            int16_t Diff_X;
            int16_t Diff_Y;
            int16_t Target_Angle;
            uint16_t mode0;
            uint16_t mode1;
            uint16_t mode2;
            uint16_t mode3;
            uint16_t Spare1;
            uint16_t Spare2;
            uint16_t Spare3;
            uint16_t Spare4;
        };
        AFLS2PLC plc_buffer_send;
       
        // -------向服务端发送的参数------



        int xian_acds_heart_beat = 0;
        int xian_acds_send_to_retrable_box_mode0 = 0;
        int xian_acds_send_to_retrable_box_mode1 = 0;
        int xian_acds_send_to_retrable_box_mode2 = 0;
        int xian_acds_send_to_retrable_box_mode3 = 0;

        // int Gantry_Position_ = 0;
        // int Trolley_Position_ = 0;
        // int Hoist_Position_ = 0;
        // int AFLS_Enable_ = 0;
        // int Unload_Enable_ = 0;
        // int Load_Enable_ = 0;
        int Heart_Beat_PLC_ = 0;
        int state0_ = 0;
        int state1_ = 0;
        int state2_ = 0;
        int state3_ = 0;
        int mode0_ = 0;
        int mode1_ = 0;
        int mode2_ = 0;
        int mode3_ = 0; 

        // int Spreader_Size_ = 0;
        // int test_recv_1 = 0;
        // int test_recv_2 = 0;
        // int v_temp_test_value = 0;
        // uint16_t inv_buf_3, inv_buf_4;
        // int zpmc_force_accs_com_enable_stat = 0;
        


        void init()
        {
            std::cout << "~~INIT~~START~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

            // zpmc_20feet.open("/root/code/xian_aqc_ws/xian_project_file/parameters/xian_aqc_dynamic_parameters.json");
            // zpmc_20feet >> zpmc_j20;
            // zpmc_20feet.close();
            // accs_ip = zpmc_j20["xian_accs_ip"];

            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_accs_ip", accs_ip);
            std::cout << "accs ip:"<< accs_ip << "  Port:" << PORT << std::endl;
            server_ip = accs_ip.c_str();
            addr.sin_family = AF_INET;
            addr.sin_port = htons(PORT);
            addr.sin_addr.s_addr = inet_addr(server_ip);

            // ros::param::set("/zpmc_unloading_parameters_node/zpmc_force_accs_com_enable_stat", 0); // 取消强制
            plc_buffer_send = {0};

            // 初始化开始时，显示连接失败
            // 无此参数，先用成员变量输出显示
            // TODO
            // ros::param::set("/zpmc_unloading_parameters_node/is_accs_connect",   0);
            is_accs_connect = false;
            connect_flag = 0;
        }

        void command_callback()
        {
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
           
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = std::max(elapsedTimeP.count(), (long)(1));

            // // ros::param::get("/xian_aqc_dynamic_parameters_server/xian_accs_ip", accs_ip);
            // server_ip = accs_ip.c_str();
            // addr.sin_family = AF_INET;
            // addr.sin_port = htons(PORT);
            // addr.sin_addr.s_addr = inet_addr(server_ip);
            

            if (
                socket_fd != -1
                && 0 == connect_flag
            )
            {
                close(socket_fd);
                socket_fd = -1;
            }

            if(0 == connect_flag)
            {
                // ------ socket ------
                std::cout << "socket 启动中......" << std::endl;
                socket_fd = socket(AF_INET, SOCK_STREAM, 0);
                if(socket_fd == -1)
                {
                    std::cout << "socket 创建失败：" << std::endl;
                    // ros::param::set("/node_ZpmcDynamicParams/is_accs_connect", 0);
                    is_accs_connect = false;
                }
                else
                {

                    tvTimeout.tv_sec = 3;
                    tvTimeout.tv_usec = 0;
                    tvTimeout_recv.tv_sec = 1;
                    tvTimeout_recv.tv_usec = 0;
                    setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &tvTimeout, sizeof(tvTimeout));
                    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tvTimeout_recv, sizeof(tvTimeout_recv));

                    // ------判断是否连接成功------
                    res = connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
                    // ------判断是否连接成功------
                    if (res != 0)
                    {
                        std::cout << "res = " << res << std::endl;
                        // std::cout << "addr.sin_addr = " << (int)(addr.sin_addr) << std::endl;
                        // std::cout << "addr.sin_port = " << (int)(addr.sin_port) << std::endl;

                        std::cout << "bind 链接失败, 再次尝试请求!" << std::endl;
                        // ros::param::set("/node_ZpmcDynamicParams/is_accs_connect", 0);
                        is_accs_connect = false;
                    }
                    else
                    {
                        connect_flag = 1; 
                        // ros::param::set("/node_ZpmcDynamicParams/is_accs_connect", 1);
                        is_accs_connect = true;
                        std::cout << "bind 链接成功!" << std::endl;
                    }
                    
                }      
            }
            
            if(1 == connect_flag)
            {
                // --------------------------------- send to accs ---------------------------------
                int iWriteCount = 0;

                plc_buffer_send.header1 = 254;
                plc_buffer_send.header2 = 254;
                plc_buffer_send.header3 = 254;
                plc_buffer_send.header4 = 254;

                uint16_t plc_buffer_send_t = sizeof(plc_buffer_send)-6;
                std::cout << "Size of send to plc: " << plc_buffer_send_t << std::endl;
                unsigned char plc_buffer_send_bytes[2];
                unsigned char plc_buffer_send_bytes_inv[2];
                Uint8ToByte(plc_buffer_send_t, plc_buffer_send_bytes);
                int plc_buffer_send_len = sizeof(plc_buffer_send_bytes);
                for(int i=0; i<plc_buffer_send_len; i++)
                {
                    plc_buffer_send_bytes_inv[i] = plc_buffer_send_bytes[plc_buffer_send_len-1-i];
                }
                uint16_t plc_buffer_send_ = ByteToUint8(plc_buffer_send_bytes_inv);
                plc_buffer_send.send_len = plc_buffer_send_;

                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_heart_beat", xian_acds_heart_beat); 
                unsigned char xian_acds_heart_beat_bytes[2];
                unsigned char xian_acds_heart_beat_bytes_inv[2];
                Uint8ToByte(xian_acds_heart_beat, xian_acds_heart_beat_bytes);
                int xian_acds_heart_beat_len = sizeof(xian_acds_heart_beat_bytes);
                for(int i=0; i<xian_acds_heart_beat_len; i++)
                {
                    xian_acds_heart_beat_bytes_inv[i] = xian_acds_heart_beat_bytes[xian_acds_heart_beat_len-1-i];
                }
                uint16_t xian_acds_heart_beat_ = ByteToUint8(xian_acds_heart_beat_bytes_inv);
                plc_buffer_send.Heart_Beat_AFLS = xian_acds_heart_beat_;
                std::cout << "Heart_Beat_AFLS: " << xian_acds_heart_beat_ << std::endl;
                
                //plc_buffer_send.Heart_Beat_AFLS = (int16_t)xian_acds_heart_beat;

                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", xian_acds_send_to_retrable_box_mode0);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", xian_acds_send_to_retrable_box_mode1);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", xian_acds_send_to_retrable_box_mode2);
                ros::param::get("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", xian_acds_send_to_retrable_box_mode3);
                
                unsigned char xian_acds_mode0_bytes[2];
                unsigned char xian_acds_mode0_bytes_inv[2];
                Uint8ToByte((int16_t)xian_acds_send_to_retrable_box_mode0, xian_acds_mode0_bytes);
                int xian_acds_mode0_len = sizeof(xian_acds_mode0_bytes);
                for(int i=0; i<xian_acds_mode0_len; i++)
                {
                    xian_acds_mode0_bytes_inv[i] = xian_acds_mode0_bytes[xian_acds_mode0_len-1-i];
                }
                uint16_t xian_acds_send_to_retrable_box_mode0_ = ByteToUint8(xian_acds_mode0_bytes_inv);                
                plc_buffer_send.mode0 = xian_acds_send_to_retrable_box_mode0_;
                
                
                unsigned char xian_acds_mode1_bytes[2];
                unsigned char xian_acds_mode1_bytes_inv[2];
                Uint8ToByte((int16_t)xian_acds_send_to_retrable_box_mode1, xian_acds_mode1_bytes);
                int xian_acds_mode1_len = sizeof(xian_acds_mode1_bytes);
                for(int i=0; i<xian_acds_mode1_len; i++)
                {
                    xian_acds_mode1_bytes_inv[i] = xian_acds_mode1_bytes[xian_acds_mode1_len-1-i];
                }
                uint16_t xian_acds_send_to_retrable_box_mode1_ = ByteToUint8(xian_acds_mode1_bytes_inv);                
                plc_buffer_send.mode1 = xian_acds_send_to_retrable_box_mode1_;
                
                
                unsigned char xian_acds_mode2_bytes[2];
                unsigned char xian_acds_mode2_bytes_inv[2];
                Uint8ToByte((int16_t)xian_acds_send_to_retrable_box_mode2, xian_acds_mode2_bytes);
                int xian_acds_mode2_len = sizeof(xian_acds_mode2_bytes);
                for(int i=0; i<xian_acds_mode2_len; i++)
                {
                    xian_acds_mode2_bytes_inv[i] = xian_acds_mode2_bytes[xian_acds_mode2_len-1-i];
                }
                uint16_t xian_acds_send_to_retrable_box_mode2_ = ByteToUint8(xian_acds_mode2_bytes_inv);                
                plc_buffer_send.mode2 = xian_acds_send_to_retrable_box_mode2_;
                
                
                
                unsigned char xian_acds_mode3_bytes[2];
                unsigned char xian_acds_mode3_bytes_inv[2];
                Uint8ToByte((int16_t)xian_acds_send_to_retrable_box_mode3, xian_acds_mode3_bytes);
                int xian_acds_mode3_len = sizeof(xian_acds_mode3_bytes);
                for(int i=0; i<xian_acds_mode3_len; i++)
                {
                    xian_acds_mode3_bytes_inv[i] = xian_acds_mode3_bytes[xian_acds_mode3_len-1-i];
                }
                uint16_t xian_acds_send_to_retrable_box_mode3_ = ByteToUint8(xian_acds_mode3_bytes_inv);                
                plc_buffer_send.mode3 = xian_acds_send_to_retrable_box_mode3_;


                std::cout << "send size:" << plc_buffer_send_t << std::endl;
                std::cout << "send to retractable box mode0:" << xian_acds_send_to_retrable_box_mode0 << std::endl;
                std::cout << "send to retractable box mode1:" << xian_acds_send_to_retrable_box_mode1 << std::endl;
                std::cout << "send to retractable box mode2:" << xian_acds_send_to_retrable_box_mode2 << std::endl;
                std::cout << "send to retractable box mode3:" << xian_acds_send_to_retrable_box_mode3 << std::endl;

                iWriteCount = write(socket_fd, (char*)&plc_buffer_send, sizeof(plc_buffer_send));
                if (iWriteCount <= 0) 
                { 
                    fail_count_send ++;
                    if (
                        fail_count_send > 5
                        && true                        
                        )
                    {
                        fail_count_send = 0;
                        fail_count_recv = 0;
                        connect_flag = 0;
                    }
                    // ros::param::set("/node_ZpmcDynamicParams/is_accs_connect", 0);
                    // is_accs_connect = false;
                    std::cout << "send data failed.....    iWriteCount = " << iWriteCount << "  fail_count_send = " << fail_count_send << std::endl;
                }
                else
                {
                    fail_count_send = 0;
                }

                // --------------------------------- send to accs ---------------------------------

   
                
                // --------------------------------- read from accs ---------------------------------
                int plc_size = read(socket_fd, (char *)&plc_buffer, sizeof(plc_buffer));//通过fd与客户端联系在一起,返回接收到的字节数
                if (plc_size <= 0) 
                { 
                    fail_count_recv ++;
                    if (
                        fail_count_recv > 50
                        && true
                        )
                    {
                        fail_count_recv = 0;
                        fail_count_send = 0;
                        connect_flag = 0;
                    }
                    // connect_flag = 0;
                    // ros::param::set("/node_ZpmcDynamicParams/is_accs_connect", 0);
                    std::cout << "get data failed.....    plc_size = " << plc_size << "  fail_count_recv = " << fail_count_recv << std::endl;
                }
                else
                {
                    fail_count_recv = 0;

                    unsigned char header1[1];
                    memcpy(header1, plc_buffer+recv_plc.header1, sizeof(header1));
                    int header1_ = HextoDec(header1, sizeof(header1));

                    unsigned char header2[1];
                    memcpy(header2, plc_buffer+recv_plc.header2, sizeof(header2));
                    int header2_ = HextoDec(header2, sizeof(header2));

                    unsigned char header3[1];
                    memcpy(header3, plc_buffer+recv_plc.header3, sizeof(header3));
                    int header3_ = HextoDec(header3, sizeof(header3));

                    unsigned char header4[1];
                    memcpy(header4, plc_buffer+recv_plc.header4, sizeof(header4));
                    int header4_ = HextoDec(header4, sizeof(header4));

                    unsigned char Heart_Beat_PLC[2];
                    memcpy(Heart_Beat_PLC, plc_buffer+recv_plc.Heart_Beat_PLC, sizeof(Heart_Beat_PLC));
                    Heart_Beat_PLC_ = HextoDec(Heart_Beat_PLC, sizeof(Heart_Beat_PLC));
                    ros::param::set("/xian_aqc_dynamic_parameters_server/xian_plc_heart_beat", Heart_Beat_PLC_);  

                    unsigned char state0[2];
                    unsigned char state0_inv[2];
                    memcpy(state0, plc_buffer+recv_plc.state0, sizeof(state0));
                    state0_ = HextoDec(state0, sizeof(state0));
                    ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state0", state0_);  

                    unsigned char state1[2];
                    unsigned char state1_inv[2];
                    memcpy(state1, plc_buffer+recv_plc.state1, sizeof(state1));
                    state1_ = HextoDec(state1, sizeof(state1));
                    ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state1", state1_);  

                    unsigned char state2[2];
                    unsigned char state2_inv[2];
                    memcpy(state2, plc_buffer+recv_plc.state2, sizeof(state2));
                    state2_ = HextoDec(state2, sizeof(state2));
                    ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state2", state2_);  

                    unsigned char state3[2];
                    unsigned char state3_inv[2];
                    memcpy(state3, plc_buffer+recv_plc.state3, sizeof(state3));
                    state3_ = HextoDec(state3, sizeof(state3));
                    ros::param::set("/xian_aqc_dynamic_parameters_server/xian_retrable_box_state3", state3_); 

                    unsigned char mode0[2];
                    unsigned char mode0_inv[2];
                    memcpy(mode0, plc_buffer+recv_plc.mode0, sizeof(mode0));
                    mode0_ = HextoDec(mode0, sizeof(mode0));
                    //ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0", mode0_); 

                    unsigned char mode1[2];
                    unsigned char mode1_inv[2];
                    memcpy(mode1, plc_buffer+recv_plc.mode1, sizeof(mode1));
                    mode1_ = HextoDec(mode1, sizeof(mode1));
                    //ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1", mode1_); 

                    unsigned char mode2[2];
                    unsigned char mode2_inv[2];
                    memcpy(mode2, plc_buffer+recv_plc.mode2, sizeof(mode2));
                    mode2_ = HextoDec(mode2, sizeof(mode2));
                    //ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2", mode2_); 

                    unsigned char mode3[2];
                    unsigned char mode3_inv[2];
                    memcpy(mode3, plc_buffer+recv_plc.mode3, sizeof(mode3));
                    mode3_ = HextoDec(mode3, sizeof(mode3));
                    //ros::param::set("/xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3", mode3_); 

                    std::cout << "Read from plc state0: " << state0_ << std::endl;
                    std::cout << "Read from plc state1: " << state1_ << std::endl;
                    std::cout << "Read from plc state2: " << state2_ << std::endl;
                    std::cout << "Read from plc state3: " << state3_ << std::endl;
                    std::cout << "Read from plc mode0: " << mode0_ << std::endl;
                    std::cout << "Read from plc mode1: " << mode1_ << std::endl;
                    std::cout << "Read from plc mode2: " << mode2_ << std::endl;
                    std::cout << "Read from plc mode3: " << mode3_ << std::endl;

                    
                    // -----------------------------------------------------------------------------------dfdsfd----------------
                    
                    /*
                    unsigned char Spreader_Size[2];
                    unsigned char Spreader_Size_inv[2];
                    memcpy(Spreader_Size, plc_buffer+recv_plc.Spreader_Size, sizeof(Spreader_Size));
                    // int Spreader_Size_len = sizeof(Spreader_Size)/sizeof(Spreader_Size[0]);
                    // for(int i=0; i<Spreader_Size_len; i++)
                    // {
                    //     Spreader_Size_inv[i] = Spreader_Size[Spreader_Size_len-1-i];
                    // }
                    // Spreader_Size_ = HextoDec(Spreader_Size_inv, sizeof(Spreader_Size_inv));
                    Spreader_Size_ = HextoDec(Spreader_Size, sizeof(Spreader_Size));

                    unsigned char Gantry_Position[4];
                    unsigned char Gantry_Position_inv[4];
                    memcpy(Gantry_Position, plc_buffer+recv_plc.Gantry_Position, sizeof(Gantry_Position));
                    // int Gantry_Position_len = sizeof(Gantry_Position)/sizeof(Gantry_Position[0]);
                    // for(int i=0; i<Gantry_Position_len; i++)
                    // {
                    //     Gantry_Position_inv[i] = Gantry_Position[Gantry_Position_len-1-i];
                    // }
                    // Gantry_Position_ = HextoDec(Gantry_Position_inv, sizeof(Gantry_Position_inv));
                    Gantry_Position_ = HextoDec(Gantry_Position, sizeof(Gantry_Position));

                    unsigned char Trolley_Position[4];
                    unsigned char Trolley_Position_inv[4];
                    memcpy(Trolley_Position, plc_buffer+recv_plc.Trolley_Position, sizeof(Trolley_Position));
                    // int Trolley_Position_len = sizeof(Trolley_Position)/sizeof(Trolley_Position[0]);
                    // for(int i=0; i<Trolley_Position_len; i++)
                    // {
                    //     Trolley_Position_inv[i] = Trolley_Position[Trolley_Position_len-1-i];
                    // }
                    // Trolley_Position_ = HextoDec(Trolley_Position_inv, sizeof(Trolley_Position_inv));
                    Trolley_Position_ = HextoDec(Trolley_Position, sizeof(Trolley_Position));

                    unsigned char Hoist_Position[4];
                    unsigned char Hoist_Position_inv[4];
                    memcpy(Hoist_Position, plc_buffer+recv_plc.Hoist_Position, sizeof(Hoist_Position));
                    // int Hoist_Position_len = sizeof(Hoist_Position)/sizeof(Hoist_Position[0]);
                    // for(int i=0; i<Hoist_Position_len; i++)
                    // {
                    //     Hoist_Position_inv[i] = Hoist_Position[Hoist_Position_len-1-i];
                    // }
                    // Hoist_Position_ = HextoDec(Hoist_Position_inv, sizeof(Hoist_Position_inv));
                    Hoist_Position_ = HextoDec(Hoist_Position, sizeof(Hoist_Position));

                    
                    unsigned char test_recv_buf[2];

                    memcpy(test_recv_buf, plc_buffer+recv_plc.TDS_Left_Control_X, sizeof(test_recv_buf));
                    test_recv_1 = HextoDec(test_recv_buf, sizeof(test_recv_buf));
                    memcpy(test_recv_buf, plc_buffer+recv_plc.TDS_Left_Control_Y, sizeof(test_recv_buf));
                    test_recv_2 = HextoDec(test_recv_buf, sizeof(test_recv_buf));



                    unsigned char AFLS_Enable[2];
                    unsigned char AFLS_Enable_inv[2];
                    memcpy(AFLS_Enable, plc_buffer+recv_plc.AFLS_Enable, sizeof(AFLS_Enable));
                    // int AFLS_Enable_len = sizeof(AFLS_Enable)/sizeof(AFLS_Enable[0]);
                    // for(int i=0; i<AFLS_Enable_len; i++)
                    // {
                    //     AFLS_Enable_inv[i] = AFLS_Enable[AFLS_Enable_len-1-i];
                    // }
                    // AFLS_Enable_ = HextoDec(AFLS_Enable_inv, sizeof(AFLS_Enable_inv));
                    AFLS_Enable_ = HextoDec(AFLS_Enable, sizeof(AFLS_Enable));

                    unsigned char Unload_Enable	[2];
                    unsigned char Unload_Enable_inv[2];
                    memcpy(Unload_Enable, plc_buffer+recv_plc.Unload_Enable, sizeof(Unload_Enable));
                    // int Unload_Enable_len = sizeof(Unload_Enable)/sizeof(Unload_Enable[0]);
                    // for(int i=0; i<Unload_Enable_len; i++)
                    // {
                    //     Unload_Enable_inv[i] = Unload_Enable[Unload_Enable_len-1-i];
                    // }
                    // Unload_Enable_ = HextoDec(Unload_Enable_inv, sizeof(Unload_Enable_inv));
                    Unload_Enable_ = HextoDec(Unload_Enable, sizeof(Unload_Enable));

                    unsigned char Load_Enable[2];
                    unsigned char Load_Enable_inv[2];
                    memcpy(Load_Enable, plc_buffer+recv_plc.Load_Enable, sizeof(Load_Enable));
                    // int Load_Enable_len = sizeof(Load_Enable)/sizeof(Load_Enable[0]);
                    // for(int i=0; i<Load_Enable_len; i++)
                    // {
                    //     Load_Enable_inv[i] = Load_Enable[Load_Enable_len-1-i];
                    // }
                    // Load_Enable_ = HextoDec(Load_Enable_inv, sizeof(Load_Enable_inv));
                    Load_Enable_ = HextoDec(Load_Enable, sizeof(Load_Enable));
                    */
                }
            }

            /*
            std::cout
            << "zpmc_communicate_with_accs_node:"                   << timeStr
            << std::endl
            << "xian_acds_heart_beat:"                              << xian_acds_heart_beat 
            << "    Gantry_Position:"                               << Gantry_Position_
            << "    Trolley_Position:"                              << Trolley_Position_
            << "    Hoist_Position:"                                << Hoist_Position_
            << "    Spreader_Size_:"                                << Spreader_Size_
            << std::endl
            << "AFLS_Enable:"                                       << AFLS_Enable_
            << "    Heart_Beat_PLC_:"                               << Heart_Beat_PLC_
            << "    Unload_Enable_:"                                << Unload_Enable_
            << "    Load_Enable_:"                                  << Load_Enable_
            << "    inv_buf_3:"                                     << inv_buf_3
            << "    inv_buf_4:"                                     << inv_buf_4
            << std::endl;
            
            if(0 == zpmc_force_accs_com_enable_stat)
            {
                ros::param::set("/zpmc_unloading_parameters_node/spreader_size",                                        Spreader_Size_);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_loading_enable",                                  Load_Enable_);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_unloading_enable",                                Unload_Enable_);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_firstlanding_enable",                             AFLS_Enable_);
            }
            else if(1 == zpmc_force_accs_com_enable_stat)
            {
                // 终端内自行设置
                // rosparam set /zpmc_unloading_parameters_node/spreader_size 1
                // rosparam set /zpmc_unloading_parameters_node/zpmc_loading_enable 1
                // rosparam set /zpmc_unloading_parameters_node/zpmc_unloading_enable 1
                // rosparam set /zpmc_unloading_parameters_node/zpmc_firstlanding_enable 1
                ;
            }
            else
            {
                ros::param::set("/zpmc_unloading_parameters_node/spreader_size",                                        Spreader_Size_);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_loading_enable",                                  Load_Enable_);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_unloading_enable",                                Unload_Enable_);
                ros::param::set("/zpmc_unloading_parameters_node/zpmc_firstlanding_enable",                             AFLS_Enable_);

                ros::param::set("/zpmc_unloading_parameters_node/zpmc_force_accs_com_enable_stat",                      0);
            }
            */
            // ros::param::set("/zpmc_unloading_parameters_node/zpmc_hist_postiton",                                   double(Hoist_Position_) / 1000.0);
            // ros::param::set("/zpmc_unloading_parameters_node/zpmc_plc_heart_beat",                                  Heart_Beat_PLC_);

        }

        int Hex2Int(char c) {
            return (c >= '0' && c <= '9') ? (c)-'0' :
                (c >= 'A' && c <= 'F') ? (c)-'A' + 10 :
                (c >= 'a' && c <= 'f') ? (c)-'a' + 10 : 0;
        }

        void Hex2Bin(const unsigned char* hex, int sz, unsigned char* out) {
            int i;
            for (i = 0; i < sz; i += 2) {
                out[i / 2] = (Hex2Int(hex[i]) << 4) | Hex2Int(hex[i + 1]);
            }
        }

        int HextoDec(const unsigned char *hex, int length) 
        { 
            int rslt = 0; 

            for(int i=0; i<length; i++) 
            { 
                rslt += (int)(hex[i])<<(8*(length-1-i)); 
            } 

            return rslt; 
        }

        void IntToByte(int value, unsigned char* bytes)
        {
            size_t length = sizeof(int);
            memset(bytes, 0, sizeof(unsigned char) * length);
            bytes[0] = (unsigned char)(0xff & value);
            bytes[1] = (unsigned char)((0xff00 & value) >> 8);
            bytes[2] = (unsigned char)((0xff0000 & value) >> 16);
            bytes[3] = (unsigned char)((0xff000000 & value) >> 24);
        }

        int ByteToInt(unsigned char* byteArray)
        {
            int value = byteArray[0] & 0xFF;
            value |= ((byteArray[1] << 8) & 0xFF00);
            value |= ((byteArray[2] << 16) & 0xFF0000);
            value |= ((byteArray[3] << 24) & 0xFF000000);
            return value;
        }


        void Uint8ToByte(uint16_t value, unsigned char* bytes)
        {
            size_t length = sizeof(uint16_t);
            memset(bytes, 0, sizeof(unsigned char) * length);
            bytes[0] = (unsigned char)(0xff & value);
            bytes[1] = (unsigned char)((0xff00 & value) >> 8);
        }

        uint16_t ByteToUint8(unsigned char* byteArray)
        {
            uint16_t value = byteArray[0] & 0xFF;
            value |= ((byteArray[1] << 8) & 0xFF00);
            return value;
        }

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"zpmc_communicate_with_accs_node");
    zpmc_CommunicationWithAccs zpmc_communicate_with_accs_node;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    zpmc_communicate_with_accs_node.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(0.1), &zpmc_CommunicationWithAccs::m_timer_HeartBeat_f, &zpmc_communicate_with_accs_node);
    zpmc_communicate_with_accs_node.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(0.1), &zpmc_CommunicationWithAccs::m_timer_Main_Func_f, &zpmc_communicate_with_accs_node);

    ros::waitForShutdown();

    // ros::spin();
    return 0;
}