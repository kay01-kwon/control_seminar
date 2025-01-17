#include "ros_wrapper/ros_plant_wrapper.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plant_node");

    ros::NodeHandle nh;

    RosPlantWrapper plant_wrapper(nh);

    plant_wrapper.run();

    return 0;
}