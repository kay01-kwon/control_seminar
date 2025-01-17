#ifndef ROS_CONTROL_HPP
#define ROS_CONTROL_HPP

#include <ros/ros.h>
#include "controller/inverted_pend_controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/state_feedback_controller.hpp"
#include "inverted_pend/State_msg.h"
#include "inverted_pend/Unpause.h"
#include "std_msgs/Float64.h"



class ROSControl{

    public:

    ROSControl(const ros::NodeHandle &nh);

    void stateCallback(const inverted_pend::State_msg::ConstPtr &msg);

    void refCallback(const inverted_pend::State_msg::ConstPtr &msg);

    void run();

    ~ROSControl();

    private:

    ros::NodeHandle nh_;

    ros::ServiceClient unpause_client_;

    ros::Subscriber des_state_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher control_pub_;

    std::unique_ptr<InvPendControl> control_;

    State des_state_;

};

#endif  // ROS_CONTROL_HPP
