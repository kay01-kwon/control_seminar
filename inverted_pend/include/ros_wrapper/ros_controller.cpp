#include "ros_controller.hpp"

ROSControl::ROSControl(const ros::NodeHandle &nh)
: nh_(nh)
{
    des_state_ << 0.0, 0.0, 0.0, 0.0;
    
    std::string control_type;

    nh_.getParam("ControlType", control_type);

    if(control_type == "PID")
    {
        double Kp, Ki, Kd;
        
        nh_.getParam("Kp", Kp);
        nh_.getParam("Ki", Ki);
        nh_.getParam("Kd", Kd);

        cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << endl;
        control_ = InvPendControl::createControl(ControlType::PID);
        static_cast<PIDControl*>(control_.get())->setGains(Kp, Ki, Kd);
    }
    else if(control_type == "StateFeedback")
    {
        double K1, K2, K3, K4;
        Gain gains;
        
        nh_.getParam("K1", K1);
        nh_.getParam("K2", K2);
        nh_.getParam("K3", K3);
        nh_.getParam("K4", K4);

        gains << K1, K2, K3, K4;

        cout << "K1: " << K1 << " K2: " << K2 << " K3: " << K3 << " K4: " << K4 << endl;
        control_ = InvPendControl::createControl(ControlType::StateFeedback);
        static_cast<StateFeedbackControl*>(control_.get())->setGains(gains);

    }
    else
    {
        ROS_ERROR("Invalid control type");
        ros::shutdown();
    }

    state_sub_ = nh_.subscribe("state", 1, &ROSControl::stateCallback, this);
    des_state_sub_ = nh_.subscribe("des_state", 1, &ROSControl::refCallback, this);
    control_pub_ = nh_.advertise<std_msgs::Float64>("/input", 1);
    unpause_client_ = nh_.serviceClient<inverted_pend::Unpause>("/unpause");

    inverted_pend::Unpause unpause_srv;
    unpause_srv.request.request = true;
    unpause_client_.call(unpause_srv);
}

void ROSControl::stateCallback(const inverted_pend::State_msg::ConstPtr &msg)
{
    double time = ros::Time::now().toSec();
    State s;

    for(size_t i = 0; i < 4; i++)
    {
        s(i) = msg->state[i];
    }

    double u = control_->computeControl(s, des_state_, time);

    std_msgs::Float64 u_msg;
    u_msg.data = u;
    control_pub_.publish(u_msg);

}

void ROSControl::refCallback(const inverted_pend::State_msg::ConstPtr &msg)
{
    for(size_t i = 0; i < 4; i++)
    {
        des_state_(i) = msg->state[i];
    }
}

void ROSControl::run()
{
    ros::spin();
}

ROSControl::~ROSControl()
{

}
