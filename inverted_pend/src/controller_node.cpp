#include "ros_wrapper/ros_controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ROSControl control(nh);

    control.run();

    return 0;
}