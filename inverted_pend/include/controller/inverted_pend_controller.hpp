#ifndef INVERTED_PEND_CONTROLLER_HPP
#define INVERTED_PEND_CONTROLLER_HPP

#include "state_def/state_def.hpp"
#include <memory>

enum class ControlType{
    PID,
    StateFeedback
};


class InvPendControl{

    public:

    InvPendControl();
    
    virtual ~InvPendControl();

    // virtual void setGains(const double &kp, const double &ki, const double &kd) = 0;

    static std::unique_ptr<InvPendControl> createControl(ControlType type);

    virtual double computeControl(const State& state, 
    const State& desired_state, const double &time) = 0;

    protected:

    double u_;

    double time_prev_;
};

#endif  // INVERTED_PEND_CONTROL_HPP