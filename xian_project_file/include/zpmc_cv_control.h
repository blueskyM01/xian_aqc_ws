#ifndef ZPMC_CV_CONTROL_H
#define ZPMC_CV_CONTROL_H

#include<stdlib.h>
#include<stdio.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<vector>
#include<math.h>
#include"chrono"
#include<numeric>
#include<ctime>
#include<fstream>


namespace zpmc
{
    cv::Mat zpmc_images_merge_row(cv::Mat A, cv::Mat B);
    cv::Mat zpmc_images_merge_col(cv::Mat A, cv::Mat B);
    cv::Mat zpmc_show_mask(cv::Mat src, cv::Mat mask);
    cv::Mat zpmc_MaskBin0_255(cv::Mat mask, int threshold);  // 图片中像素小于threshold的设为0，反之为255，输入图像为RGB图像
    char* zpmc_get_stystem_time();
    std::vector<std::vector<double>> zpmc_marker_target_points_get(cv::Mat src, int L_Type_request, double scale_lower, double scale_upper, 
                                                                      double point_w_a, double point_w_b, int area_threshold, 
                                                                      double container_point_x, double container_point_y, 
                                                                      int distance_threshold);
    std::vector<std::vector<cv::Point>> zpmc_find_contours(cv::Mat src);
    std::vector<std::vector<cv::Point>> zpmc_interpolation_contours(const std::vector<std::vector<cv::Point>> & _contours, int norm_steps);
    double* zpmc_get_points_feature(const std::vector<cv::Point> & cornerLine_pixel_group, double scale_lower, double scale_upper);
    int* zpmc_LType_recognition(const std::vector<cv::Point> & cornerLine_pixel_group, double x_min, double y_min, double x_max, double y_max, int cornerLineAera, double scale_lower, double scale_upper);
    cv::Point zpmc_LType_corner(std::vector<cv::Point> contour, int L_Type, cv::Point2f xy_1, cv::Point2f xy_2);
    cv::Mat zpmc_show_L_marker_mask_result(cv::Mat src, std::vector<std::vector<double>>, int L_Type_request);

    void xresize(const cv::Mat &src, cv::Mat &des, cv::Size size); // 最近邻缩放算法

    cv::Point* crop_area(int cx, int cy, int crop_w, int crop_h, int src_w, int src_h);

    class zpmc_ControlAlgorithm
    {
        public:
            
        zpmc_ControlAlgorithm();
        ~zpmc_ControlAlgorithm();
        double PID_realize(double error, int history_len);
        double firstOrderFilter(double in_data);
        double zpmc_sqrtFilter(double err); 
            
        double err_current;                
        double err_last;            
        double Kp = 1.0;
        double Ki = 1.0;
        double Kd = 1.0;            
        double Uk = 1.0;
        std::vector<double> error_history;

        double alph = 0.4;
        double zpmc_final = 0;

        private:
            
    };

    class zpmc_SpreaderLogicControl
    {
        public:

        zpmc_SpreaderLogicControl();
        ~zpmc_SpreaderLogicControl();

        double* zpmc_4points_spreader_eror(double tl_ex, double tr_ex, double bl_ex, double br_ex, double tl_ey, double tr_ey, double bl_ey, double br_ey);
        double* zpmc_3points_spreader_eror(int not_detected_point, double tl_ex, double tr_ex, double bl_ex, double br_ex, double tl_ey, double tr_ey, double bl_ey, double br_ey);
        double* zpmc_2points_spreader_eror(int not_detected_2point, double tl_ex, double tr_ex, double bl_ex, double br_ex, double tl_ey, double tr_ey, double bl_ey, double br_ey);
        private:

    };

    class SimpleLog
    {
        private:
            tm mPCreateFileTime;
            
            
            void CheckCreate();
        public:
            SimpleLog();
            ~SimpleLog();
            void Write(std::string log);
            void Write(std::string log, double value);
            void CreateFile(std::string logdir);
            void close();
        
        // var
        std::ofstream mOutfile;
    
    };

}

#endif