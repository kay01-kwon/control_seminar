#include "pid_controller.hpp"

PIDControl::PIDControl()
:error_(0.0), prev_error_(0.0), integral_(0.0)
{
    cout<<"PID Controller created"<<endl;
}

PIDControl::~PIDControl()
{
}

void PIDControl::setGains(const double &kp, 
const double &ki, const double &kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    cout<<"Kp: "<<kp_<<endl;
    cout<<"Ki: "<<ki_<<endl;
    cout<<"Kd: "<<kd_<<endl;
}

double PIDControl::computeControl(const State &state, const State &des_state,
const double &time)
{

    double dt = time - time_prev_;

    if(dt >= 0.01)
    {
        dt = 0.01;
    }
    else if(dt <= 0.001)
    {
        dt = 0.001;
    }

    error_ = des_state(0) - state(0);
    integral_ += error_*dt;
    u_ = kp_*error_ + ki_*integral_ + kd_*(error_ - prev_error_)/dt;

    prev_error_ = error_;
    time_prev_ = time;

    return u_;
}
