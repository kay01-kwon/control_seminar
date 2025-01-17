#include "ros_plant_wrapper.hpp"

RosPlantWrapper::RosPlantWrapper(const NodeHandle &nh)
:nh_(nh)
{
    double m, M, l;
    
    double theta0, dthetadt0, x0, dxdt0;

    nh_.getParam("m", m);
    nh_.getParam("M", M);
    nh_.getParam("l", l);
    nh_.getParam("theta0", theta0);

    state_ << theta0, 0, 0, 0;

    plant_unpase_server_ = nh_.advertiseService("/unpause",
    &RosPlantWrapper::unpause_callback, this);

    plant_input_sub_ = nh_.subscribe("/input", 1, 
    &RosPlantWrapper::plant_input_callback, 
    this);

    plant_state_pub_ = nh_.advertise<State_msg>("/state", 1);

    ode_solver_ptr = new OdeRk4Custom<State>();

    plant_dynamics_ptr = new PlantDynamics({m, M, l});


}

RosPlantWrapper::~RosPlantWrapper()
{
    delete ode_solver_ptr;
    delete plant_dynamics_ptr;
}

bool RosPlantWrapper::unpause_callback(Unpause::Request &req, 
Unpause::Response &res)
{
    if(req.request == true)
    {
        paused_ = false;
        return true;
    }
    return false;
}

void RosPlantWrapper::plant_input_callback(const std_msgs::Float64::ConstPtr &msg)
{
    plant_dynamics_ptr->set_input(msg->data);
}


void RosPlantWrapper::publish_state(const State &state)
{
    State_msg state_msg;

    state_msg.stamp = ros::Time::now();

    for(size_t i = 0; i < 4; i++)
    {
        state_msg.state[i] = state(i);
    }

    plant_state_pub_.publish(state_msg);

}

void RosPlantWrapper::run()
{
    ros::Rate loop_rate{100};

    State state;

    while(ros::ok())
    {

        double time_now = ros::Time::now().toSec();

        if(paused_)
        {
            time_prev_ = time_now;
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        if(first_run_)
        {
            dt_ = 0;
            first_run_ = false;
        }
        else
        {
            dt_ = time_now - time_prev_;
        }

        ode_solver_ptr->do_step(
            [this](const State &s, State &s_dot, const double &t)
            {
                plant_dynamics_ptr->dynamics(s, s_dot, t);
            }
        , state_, time_now, dt_);

        // cout<<"State: "<<state_.transpose()<<endl;

        publish_state(state_);

        time_prev_ = time_now;

        ros::spinOnce();

        loop_rate.sleep();

    }
}