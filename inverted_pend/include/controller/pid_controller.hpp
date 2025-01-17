#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "inverted_pend_controller.hpp"

class PIDControl : public InvPendControl{
    public:

    PIDControl();

    ~PIDControl();

    void setGains(const double& kp, const double& ki, const double& kd);

    double computeControl(const State& state, 
    const State& des_state, const double& time) override;

    private:
    State desired_state_;

    double kp_{1.0};
    double ki_{0.0};
    double kd_{0.01};
    
    double error_;
    double prev_error_;
    double integral_;
};


#endif  // PID_CONTROL_HPP