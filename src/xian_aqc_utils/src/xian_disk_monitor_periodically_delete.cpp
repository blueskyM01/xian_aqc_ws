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

#include <dirent.h>
#include <sys/stat.h>
#include <sys/statfs.h>

#include <vector>
#include <cstdlib>
#include <memory>
#include <unistd.h>
#include <iostream>   
#include <string> 

#include <chrono>

// using namespace std::chrono_literals;

// 需要在CMakeList.txt中增加如下内容
// set(CMAKE_CXX_STANDARD 14)
// set(CMAKE_CXX_STANDARD_REQUIRED ON)
// set(CMAKE_CXX_EXTENSIONS OFF)
using namespace std::literals::chrono_literals;


class zpmc_DiskMonitorPeriodicallyDelete
{
    public:
        zpmc_DiskMonitorPeriodicallyDelete()
        {
            std::cout << "xian_disk_monitor_periodically_delete:  节点已启动" << timeStr << std::endl;
            // 创建一个ROS节点句柄

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
            // ros::param::get("/zpmc_unloading_parameters_node/xian_disk_monitor_periodically_delete_heart_beat", xian_disk_monitor_periodically_delete_heart_beat); // 自行替换
            counter = counter > 1000 ? 0 : (counter + 1);
            // std::cout << "counter standalone: " << counter << std::endl;
            ros::param::set("/zpmc_unloading_parameters_node/xian_disk_monitor_periodically_delete_heart_beat", counter);  // 自行替换
        }

    private:

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
 
        std::string timeStr;


        int xian_disk_monitor_periodically_delete_heart_beat = 0;
        double xian_disk_monitor_periodically_delete_fps = 0.0;


        double xian_disk_area_left = 101.0;

        // ----------periodocally delete-------
        unsigned long long zpmc_td = 10240; // 10GB

        // 查看指定文件夹的剩余空间
        std::string log_dir = "/root/code/log";

        std::string log_dir_ar[9];

        void command_callback()
        {
            std::cout
            << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            << std::endl;


            log_dir_ar[0] = "/root/code/log/origin_data_saving";

            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();
            timeStr = zpmc::zpmc_get_stystem_time();
            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = std::max(elapsedTimeP.count(), (long)(1));

            ros::param::get("/zpmc_unloading_parameters_node/xian_disk_monitor_periodically_delete_fps",        xian_disk_monitor_periodically_delete_fps);
            ros::param::get("/zpmc_unloading_parameters_node/xian_disk_area_left",                              xian_disk_area_left);

             // 用于获取磁盘剩余空间
            struct statfs diskInfo;
            statfs(log_dir.c_str(), &diskInfo);

            unsigned long long blocksize                = diskInfo.f_bsize;	//每个block里包含的字节数
            // unsigned long long totalsize                = blocksize * diskInfo.f_blocks; 	//总的字节数，f_blocks为block的数目

            // printf("Total_size = %llu B                 = %llu KB = %llu MB = %llu GB\n", 
            // 	                                            totalsize, totalsize>>10, totalsize>>20, totalsize>>30);
            
            // unsigned long long freeDisk                 = diskInfo.f_bfree * blocksize;	//剩余空间的大小
            unsigned long long availableDisk            = diskInfo.f_bavail * blocksize; 	//可用空间大小

            xian_disk_area_left = double(availableDisk>>30);
            std::cout << "availableDisk:" << xian_disk_area_left << " GB" << std::endl;

            if((availableDisk>>20) < zpmc_td)
            {
                struct stat s;
                std::vector<std::string> files;

                // 遍历log_dir_ar[]所有文件夹并将文件存入files
                for (int j = 0; j < 1; j++)
                {
                    getFiles(log_dir_ar[j], files);
                }
                // sort(files.begin(), files.end());
                // 2023.07.05 按裸文件名排序
                sort(files.begin(), files.end(), m_fComparePureFileName);

                std::cout << "files.size():" << files.size() <<std::endl;
                if(files.size() >=1)
                {
                    for (int i = 0; i < 1; i++) 
                    {
                        std::string file_c = files[i];
                        char* file = (char*)file_c.data();
                        // cout << file << endl;
                        if(stat(file,&s)==0)
                        {
                            if(s.st_mode & S_IFDIR)
                            {
                                // std::cout<<"it's a directory"<<std::endl;
                                rm_dir(file);
                                std::cout<<"Delete " << file << " successful!" <<std::endl;
                            }
                            else if(s.st_mode & S_IFREG)
                            {
                                // std::cout<<"it's a file"<<std::endl;
                                if(remove(file)==0)
                                {
                                    std::cout<<"Delete " << file << " successful!" <<std::endl;
                                }
                                
                            }
                            else
                            {
                                std::cout<<"not file not directory"<<std::endl;
                            }
                        }
                        else
                        {
                            std::cout<<"error, doesn't exist"<<std::endl;
                        }
                    }
                }

            }

            std::cout
            // << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            // << std::endl
            << "xian_disk_monitor_periodically_delete:"                    << timeStr
            << std::endl
            << "xian_disk_monitor_periodically_delete_heart_beat:"         << xian_disk_monitor_periodically_delete_heart_beat 
            << "    xian_disk_monitor_periodically_delete_fps:"            << xian_disk_monitor_periodically_delete_fps
            << "    xian_disk_area_left:"                                  << xian_disk_area_left
            << std::endl;

            // ros::param::set("/zpmc_unloading_parameters_node/xian_disk_monitor_periodically_delete_heart_beat",    counter);
            ros::param::set("/zpmc_unloading_parameters_node/xian_disk_monitor_periodically_delete_fps",           1000.0 / timediff);
            ros::param::set("/zpmc_unloading_parameters_node/xian_disk_area_left",                                 xian_disk_area_left);
        }


        void getFiles(const std::string &root, std::vector<std::string> &files) 
        {
            DIR *pDir; //指向根目录结构体的指针
            struct dirent *ptr; //dirent结构体指针，具体结构看开头的注释
            // 使用dirent.h下的opendir()打开根目录，并返回指针
            if (!(pDir = opendir(root.c_str()))) {
                return;
            }
            // 使用dirent.h下的readdir逐个读取root下的文件
            while ((ptr = readdir(pDir)) != nullptr) {
                // 这里我理解他的指针应该是自动会指向到下一个文件，所以不用写指针的移动
                std::string sub_file = root + "/" + ptr->d_name; // 当前指针指向的文件名
                // if (ptr->d_type != 8 && ptr->d_type != 4) { // 递归出口，当不是普通文件（8）和文件夹（4）时退出递归
                //   return;
                // }
                // 普通文件直接加入到files
                // if (ptr->d_type == 8)
                // {
                // 相当于将命令下使用ls展示出来的文件中除了. 和 ..全部保存在files中
                // 当然下面可以写各种字符串的筛选逻辑，比如只要后缀有.jpg图片
                if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) 
                {
                    files.push_back(sub_file);
                } 
                // } // 当前文件为文件夹(4)类型，那就以当前文件为root进行递归吧！
                // else if (ptr->d_type == 4)
                // {
                //   if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
                //     getFiles(sub_file, files);
                //   }
                // }
            }
            // 关闭根目录
            closedir(pDir);
        }

        // 2023.07.05 按裸文件名排序
        // 如按完整文件名排序，会出现 /root/code/log/firstlanding/2023-00-00-00-00-00-000-firstlanding.log   <   /root/code/log/loading/1999-00-00-00-00-00-000-loading.log
        static bool m_fComparePureFileName(std::string i1, std::string i2)
        {
            std::string::size_type iPos_1 = i1.find_last_of('/') + 1;
            std::string::size_type iPos_2 = i2.find_last_of('/') + 1;
            std::string p1 = i1.substr(iPos_1, i1.length() - iPos_1);
            std::string p2 = i2.substr(iPos_2, i2.length() - iPos_2);
            return (p1 < p2);
        }

        void error_quit( const char *msg )
        {
            perror( msg );
            exit( -1 );
        }
        void change_path( const char *path )
        {
            printf( "Leave %s Successed . . .\n", getcwd( NULL, 0 ) );
            if ( chdir( path ) == -1 )
                error_quit( "chdir" );
            printf( "Entry %s Successed . . .\n", getcwd( NULL, 0 ) );
        }
        void rm_dir( const char *path )
        {
            DIR    *dir;
            struct dirent  *dirp;
            struct stat  buf;
            char    *p = getcwd( NULL, 0 );
            if ( (dir = opendir( path ) ) == NULL )
                error_quit( "OpenDir" );
            change_path( path );
            while ( dirp = readdir(dir) )
            {
                if ( (strcmp( dirp->d_name, "." ) == 0) || (strcmp( dirp->d_name, ".." ) == 0) )
                continue;
                if ( stat( dirp->d_name, &buf ) == -1 )
                error_quit( "stat" );
                if ( S_ISDIR( buf.st_mode ) )
                {
                rm_dir( dirp->d_name );
                /*if(rmdir(dirp->d_name)==-1)
                *  error_quit("rmdir");
                * printf("rm %s Successed . . .\n",dirp->d_name);*/
                continue;
                }
                if ( remove( dirp->d_name ) == -1 )
                error_quit( "remove" );
                printf( "rm %s Successed . . .\n", dirp->d_name );
            }
            closedir( dir );
            change_path( p );
            if ( rmdir( path ) == -1 )
                error_quit( "rmdir" );
            printf( "rm %s Successed . . .\n", path );
        }


};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_disk_monitor_periodically_delete");
    zpmc_DiskMonitorPeriodicallyDelete xian_disk_monitor_periodically_delete;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_disk_monitor_periodically_delete.m_timer_Main_Func = nh_2.createWallTimer(ros::WallDuration(1.0), &zpmc_DiskMonitorPeriodicallyDelete::m_timer_Main_Func_f, &xian_disk_monitor_periodically_delete);
    xian_disk_monitor_periodically_delete.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &zpmc_DiskMonitorPeriodicallyDelete::m_timer_HeartBeat_f, &xian_disk_monitor_periodically_delete);

    ros::waitForShutdown();
    // ros::spin();
    return 0;
}