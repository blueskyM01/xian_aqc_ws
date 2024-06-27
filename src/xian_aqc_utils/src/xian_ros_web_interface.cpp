#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<sys/types.h>
#include "xian_msg_pkg/xian_ros_web_interface.h"
#include "zpmc_cv_control.h"


class Xian_RosWebInterface
{
    public:
        Xian_RosWebInterface()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            command_publisher_show = nh.advertise<xian_msg_pkg::xian_ros_web_interface>("ros_param_visualization", 1);
        }        
        ros::WallTimer m_timer_HeartBeat;
        ros::WallTimer m_timer_Publisher;

        void m_timer_HeartBeat_f(const ros::WallTimerEvent& event)
        {
            counter = counter > 1000 ? 0 : (counter + 1);
            std::cout << "xian_xian_ros_web_interface_heart_beat: " << counter << std::endl;
        }

        void m_timer_Publisher_f(const ros::WallTimerEvent& event)
        {
            timeStr = zpmc::zpmc_get_stystem_time();
            pre_time = cur_time;
            cur_time = std::chrono::high_resolution_clock::now();

            ros::param::get("/xian_aqc_dynamic_parameters_server/area_threshold", area_threshold); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/identifed_container_corner_distance", identifed_container_corner_distance); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/point_w_a", point_w_a); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/point_w_b", point_w_b); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/point_w_a", scale_lower); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/point_w_b", scale_upper); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_x", xian_tl_container_point_x); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_y", xian_tl_container_point_y); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_x", xian_tr_container_point_x); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_y", xian_tr_container_point_y); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_x", xian_bl_container_point_x); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_y", xian_bl_container_point_y); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_br_container_point_x", xian_br_container_point_x); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_br_container_point_y", xian_br_container_point_y); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_bl_1", xian_crop_bais_x_bl_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_bl_2", xian_crop_bais_x_bl_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_br_1", xian_crop_bais_x_br_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_br_2", xian_crop_bais_x_br_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tl_1", xian_crop_bais_x_tl_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tl_2", xian_crop_bais_x_tl_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tr_1", xian_crop_bais_x_tr_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_x_tr_2", xian_crop_bais_x_tr_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_bl_1", xian_crop_bais_y_bl_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_bl_2", xian_crop_bais_y_bl_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_br_1", xian_crop_bais_y_br_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_br_2", xian_crop_bais_y_br_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tl_1", xian_crop_bais_y_tl_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tl_2", xian_crop_bais_y_tl_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tr_1", xian_crop_bais_y_tr_1); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_crop_bais_y_tr_2", xian_crop_bais_y_tr_2); 
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_is_calibrate_flag",xian_is_calibrate_flag);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_aqc_dynamic_param_node_heart_beat",xian_aqc_dynamic_param_node_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_plc_heart_beat",xian_plc_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_camera_driver_heat_beat",xian_camera_driver_heat_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_container_corner_cop_process_heart_beat",xian_container_corner_cop_process_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_keypoints_recognition_heart_beat",xian_keypoints_recognition_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_get_cell_guide_roi_heart_beat",xian_get_cell_guide_roi_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_recognition_heat_beat",xian_cell_guide_recognition_heat_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_mask_resize_heart_beat",xian_cell_guide_mask_resize_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_point_identification_heart_beat",xian_cell_guide_point_identification_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_error_restart_log_heart_beat",xian_error_restart_log_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_trolley_side_client_ros_heart_beat",xian_trolley_side_client_ros_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_spreader_side_server_ros_heart_beat",xian_spreader_side_server_ros_heart_beat);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_camera_sensor_fps",xian_camera_sensor_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_container_corner_cop_process_fps",xian_container_corner_cop_process_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_keypoints_recognition_fps",xian_keypoints_recognition_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_get_cell_guide_roi_fps",xian_get_cell_guide_roi_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_recognition_fps",xian_cell_guide_recognition_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_mask_resize_fps",xian_cell_guide_mask_resize_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_cell_guide_point_identification_fps",xian_cell_guide_point_identification_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_error_restart_log_fps",xian_error_restart_log_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_trolley_side_client_ros_fps",xian_trolley_side_client_ros_fps);
            ros::param::get("/xian_aqc_dynamic_parameters_server/xian_spreader_side_server_ros_fps",xian_spreader_side_server_ros_fps);

            web_show.area_threshold = area_threshold;
            web_show.identifed_container_corner_distance = identifed_container_corner_distance;
            web_show.point_w_a = point_w_a;
            web_show.point_w_b = point_w_b;
            web_show.scale_lower = scale_lower;
            web_show.scale_upper = scale_upper;
            web_show.xian_tl_container_point_x = xian_tl_container_point_x;
            web_show.xian_tl_container_point_y = xian_tl_container_point_y;
            web_show.xian_tr_container_point_x = xian_tr_container_point_x;
            web_show.xian_tr_container_point_y = xian_tr_container_point_y;
            web_show.xian_bl_container_point_x = xian_bl_container_point_x;
            web_show.xian_bl_container_point_y = xian_bl_container_point_y;
            web_show.xian_br_container_point_x = xian_br_container_point_x;
            web_show.xian_br_container_point_y = xian_br_container_point_y;
            web_show.xian_crop_bais_x_bl_1 = xian_crop_bais_x_bl_1;
            web_show.xian_crop_bais_x_bl_2 = xian_crop_bais_x_bl_2;
            web_show.xian_crop_bais_x_br_1 = xian_crop_bais_x_br_1;
            web_show.xian_crop_bais_x_br_2 = xian_crop_bais_x_br_2;
            web_show.xian_crop_bais_x_tl_1 = xian_crop_bais_x_tl_1;
            web_show.xian_crop_bais_x_tl_2 = xian_crop_bais_x_tl_2;
            web_show.xian_crop_bais_x_tr_1 = xian_crop_bais_x_tr_1;
            web_show.xian_crop_bais_x_tr_2 = xian_crop_bais_x_tr_2;
            web_show.xian_crop_bais_y_bl_1 = xian_crop_bais_y_bl_1;
            web_show.xian_crop_bais_y_bl_2 = xian_crop_bais_y_bl_2;
            web_show.xian_crop_bais_y_br_1 = xian_crop_bais_y_br_1;
            web_show.xian_crop_bais_y_br_2 = xian_crop_bais_y_br_2;
            web_show.xian_crop_bais_y_tl_1 = xian_crop_bais_y_tl_1;
            web_show.xian_crop_bais_y_tl_2 = xian_crop_bais_y_tl_2;
            web_show.xian_crop_bais_y_tr_1 = xian_crop_bais_y_tr_1;
            web_show.xian_crop_bais_y_tr_2 = xian_crop_bais_y_tr_2;
            web_show.xian_is_calibrate_flag = xian_is_calibrate_flag;
            web_show.xian_aqc_dynamic_param_node_heart_beat = xian_aqc_dynamic_param_node_heart_beat;
            web_show.xian_plc_heart_beat = xian_plc_heart_beat;
            web_show.xian_camera_driver_heat_beat = xian_camera_driver_heat_beat;
            web_show.xian_container_corner_cop_process_heart_beat = xian_container_corner_cop_process_heart_beat;
            web_show.xian_keypoints_recognition_heart_beat = xian_keypoints_recognition_heart_beat;
            web_show.xian_get_cell_guide_roi_heart_beat = xian_get_cell_guide_roi_heart_beat;
            web_show.xian_cell_guide_recognition_heat_beat = xian_cell_guide_recognition_heat_beat;
            web_show.xian_cell_guide_mask_resize_heart_beat = xian_cell_guide_mask_resize_heart_beat;
            web_show.xian_cell_guide_point_identification_heart_beat = xian_cell_guide_point_identification_heart_beat;
            web_show.xian_error_restart_log_heart_beat = xian_error_restart_log_heart_beat;
            web_show.xian_trolley_side_client_ros_heart_beat = xian_trolley_side_client_ros_heart_beat;
            web_show.xian_spreader_side_server_ros_heart_beat = xian_spreader_side_server_ros_heart_beat;
            web_show.xian_camera_sensor_fps = xian_camera_sensor_fps;
            web_show.xian_container_corner_cop_process_fps = xian_container_corner_cop_process_fps;
            web_show.xian_keypoints_recognition_fps = xian_keypoints_recognition_fps;
            web_show.xian_get_cell_guide_roi_fps = xian_get_cell_guide_roi_fps;
            web_show.xian_cell_guide_recognition_fps = xian_cell_guide_recognition_fps;
            web_show.xian_cell_guide_mask_resize_fps = xian_cell_guide_mask_resize_fps;
            web_show.xian_cell_guide_point_identification_fps = xian_cell_guide_point_identification_fps;
            web_show.xian_error_restart_log_fps = xian_error_restart_log_fps;
            web_show.xian_trolley_side_client_ros_fps = xian_trolley_side_client_ros_fps;
            web_show.xian_spreader_side_server_ros_fps = xian_spreader_side_server_ros_fps;
            command_publisher_show.publish(web_show);

            elapsedTimeP = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pre_time);
            timediff = elapsedTimeP.count();
            std::cout << "FPS: " << 1000.0 / timediff << std::endl;
        } 

    private:
        ros::Publisher command_publisher_show;

        int area_threshold = 0;
        int identifed_container_corner_distance = 0;
        int point_w_a = 0;
        int point_w_b = 0;
        int scale_lower = 0;
        int scale_upper = 0;
        int xian_tl_container_point_x = 0;
        int xian_tl_container_point_y = 0;
        int xian_tr_container_point_x = 0;
        int xian_tr_container_point_y = 0;
        int xian_bl_container_point_x = 0;
        int xian_bl_container_point_y = 0;
        int xian_br_container_point_x = 0;
        int xian_br_container_point_y = 0;
        int xian_crop_bais_x_bl_1 = 0;
        int xian_crop_bais_x_bl_2 = 0;
        int xian_crop_bais_x_br_1 = 0;
        int xian_crop_bais_x_br_2 = 0;
        int xian_crop_bais_x_tl_1 = 0;
        int xian_crop_bais_x_tl_2 = 0;
        int xian_crop_bais_x_tr_1 = 0;
        int xian_crop_bais_x_tr_2 = 0;
        int xian_crop_bais_y_bl_1 = 0;
        int xian_crop_bais_y_bl_2 = 0;
        int xian_crop_bais_y_br_1 = 0;
        int xian_crop_bais_y_br_2 = 0;
        int xian_crop_bais_y_tl_1 = 0;
        int xian_crop_bais_y_tl_2 = 0;
        int xian_crop_bais_y_tr_1 = 0;
        int xian_crop_bais_y_tr_2 = 0;
        int xian_is_calibrate_flag = 0;
        int xian_aqc_dynamic_param_node_heart_beat = 0;
        int xian_plc_heart_beat = 0;
        int xian_camera_driver_heat_beat = 0;
        int xian_container_corner_cop_process_heart_beat = 0;
        int xian_keypoints_recognition_heart_beat = 0;
        int xian_get_cell_guide_roi_heart_beat = 0;
        int xian_cell_guide_recognition_heat_beat = 0;
        int xian_cell_guide_mask_resize_heart_beat = 0;
        int xian_cell_guide_point_identification_heart_beat = 0;
        int xian_error_restart_log_heart_beat = 0;
        int xian_trolley_side_client_ros_heart_beat = 0;
        int xian_spreader_side_server_ros_heart_beat = 0;
        double xian_camera_sensor_fps = 0;
        double xian_container_corner_cop_process_fps = 0;
        double xian_keypoints_recognition_fps = 0;
        double xian_get_cell_guide_roi_fps = 0;
        double xian_cell_guide_recognition_fps = 0;
        double xian_cell_guide_mask_resize_fps = 0;
        double xian_cell_guide_point_identification_fps = 0;
        double xian_error_restart_log_fps = 0;
        double xian_trolley_side_client_ros_fps = 0;
        double xian_spreader_side_server_ros_fps = 0;

        std::chrono::_V2::system_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point pre_time = std::chrono::high_resolution_clock::now();
        std::chrono::__enable_if_is_duration<std::chrono::duration<long int, std::ratio<1, 1000>>> elapsedTimeP;
        double timediff = 1.0;
        int counter = 0;
        std::string timeStr;
        xian_msg_pkg::xian_ros_web_interface web_show;
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_ros_web_interface_node");
    Xian_RosWebInterface xian_ros_web_interface;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_ros_web_interface.m_timer_HeartBeat = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_RosWebInterface::m_timer_HeartBeat_f, &xian_ros_web_interface);
    xian_ros_web_interface.m_timer_Publisher = nh_2.createWallTimer(ros::WallDuration(1.0), &Xian_RosWebInterface::m_timer_Publisher_f, &xian_ros_web_interface);

    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}
