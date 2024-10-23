#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <thread>
#include <chrono> 

void processStream(const std::string& pipeline, const std::string& topic) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic, 1);

    cv::VideoCapture cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open GStreamer pipeline: %s", pipeline.c_str());
        return;
    }

    ros::Rate loop_rate(60);

    while (ros::ok()) {
        if (!cap.grab()) {
            ROS_WARN("Failed to grab frame, skipping...");
            continue;
        }

        cv::Mat frame;
        if (!cap.retrieve(frame)) {
            ROS_WARN("Failed to retrieve frame, skipping...");
            continue;
        }

        if (frame.empty()) {
            ROS_WARN("Frame is empty, skipping...");
            continue;
        }

        // auto start = std::chrono::high_resolution_clock::now();

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // auto end = std::chrono::high_resolution_clock::now();

        // std::chrono::duration<double> elapsed = end - start;

        // ROS_INFO("Time taken to convert CvImage to ImageMsg: %f seconds", elapsed.count());

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtsp_to_ros");
    ros::NodeHandle nh("~"); // 使用私有命名空间来获取参数

    std::string gstreamer_pipeline;
    std::string topic_name;

    // 获取参数
    nh.getParam("pipeline", gstreamer_pipeline);
    nh.getParam("topic", topic_name);

    std::thread thread0(processStream, gstreamer_pipeline, topic_name);

    thread0.join();
    return 0;
}
