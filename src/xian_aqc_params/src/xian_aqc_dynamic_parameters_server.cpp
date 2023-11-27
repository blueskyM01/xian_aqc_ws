#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "xian_aqc_params/xian_aqc_dynamic_parametersConfig.h"
//define call back function
void paramCallback(xian_aqc_params::xian_aqc_dynamic_parametersConfig& config,uint32_t level)
{
    ROS_INFO("Request");
}
int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_aqc_dynamic_parameters_server");
    //create node handle
    dynamic_reconfigure::Server<xian_aqc_params::xian_aqc_dynamic_parametersConfig> server;
    dynamic_reconfigure::Server<xian_aqc_params::xian_aqc_dynamic_parametersConfig>::CallbackType f;
    f = boost::bind(&paramCallback,_1,_2);
    server.setCallback(f);
    int counter = 0;
    int xian_aqc_dynamic_param_node_heart_beat = 0;
    while(ros::ok())
    {
        usleep(1000 * 1000); // 1000 ms
        ros::param::set("/xian_aqc_dynamic_parameters_server/xian_aqc_dynamic_param_node_heart_beat", counter); // 原始检测结果
        ros::param::get("/xian_aqc_dynamic_parameters_server/xian_aqc_dynamic_param_node_heart_beat", xian_aqc_dynamic_param_node_heart_beat);
        std::cout << "xian_aqc_dynamic_param_node_heart_beat:" << xian_aqc_dynamic_param_node_heart_beat << std::endl;
        if(counter > 1000)
        {
            counter = 0;
        }

        counter ++;
    }
    ros::spin();
    return 0;
}