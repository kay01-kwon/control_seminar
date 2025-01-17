#include "inverted_pend_controller.hpp"
#include "pid_controller.hpp"

InvPendControl::InvPendControl()
{
}

InvPendControl::~InvPendControl()
{
}

void InvPendControl::setGains(const double &kp, const double &ki, 
const double &kd)
{
}

std::unique_ptr<InvPendControl> InvPendControl::createControl(ControlType type)
{
    if(type == ControlType::PID)
    {
        return std::make_unique<PIDControl>();
    }
    else if(type == ControlType::StateFeedback)
    {
        // return new StateFeedbackControl();
    }

    return nullptr;
}