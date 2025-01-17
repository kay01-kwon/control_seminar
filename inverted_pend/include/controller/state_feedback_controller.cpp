#include "state_feedback_controller.hpp"

StateFeedbackControl::StateFeedbackControl()
{
}

StateFeedbackControl::~StateFeedbackControl()
{
}

void StateFeedbackControl::setGains(const Gain &gains)
{
    gains_ = gains;
}

double StateFeedbackControl::computeControl(const State &state, const State &des_state, const double &time)
{
    // Compute the control input using state feedback control
    // u = -K(x - x_des)
    // where K is the gain matrix
    // x is the current state
    // x_des is the desired state
    // u is the control input
    double u;
    u = -gains_* (state - des_state);
    return u;
}
