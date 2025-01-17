#ifndef STATE_FEEDBACK_CONTROLLER_HPP
#define STATE_FEEDBACK_CONTROLLER_HPP

#include "inverted_pend_controller.hpp"

class StateFeedbackControl : public InvPendControl{
    public:

    StateFeedbackControl();

    ~StateFeedbackControl();

    void setGains(const Gain &gains);

    double computeControl(const State& state, 
    const State& des_state, const double& time) override;

    private:
    State desired_state_;

    Gain gains_;

};

#endif // STATE_FEEDBACK_CONTROLLER_HPP