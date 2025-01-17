#include "plant_dynamics.hpp"

PlantDynamics::PlantDynamics(const PlantParams &params)
: params_(params)
{
    cout << "PlantDynamics object created" << endl;

    cout << "Initial state: " << state_.transpose() << endl;

    cout << "Plant parameters: " << endl;

    cout << "m: " << params_.m << endl;

    cout << "M: " << params_.M << endl;

    cout << "l: " << params_.l << endl;
}

void PlantDynamics::set_input(const double &u)
{
    u_ = u;
}

void PlantDynamics::get_state(State &state) const
{
    state = state_;
}

void PlantDynamics::get_time(double &time) const
{
    time = time_;
}

void PlantDynamics::dynamics(const State &state,
State &state_dot, const double &time)
{
    // Unpack state
    double theta = state(0);
    double dthetadt = state(1);
    double x = state(2);
    double dxdt = state(3);

    // Unpack parameters
    double m = params_.m;
    double M = params_.M;
    double l = params_.l;

    double ddtheta_dt2, ddx_dt2;


    double I_p_f = 1.0/3.0*m*l*l;
    double Delta;

    Mat21 u_vec, y_vec;
    Mat22 T;

    u_vec << 0.5*( m*gravity_*l*sin(theta) 
    - m*dxdt*l*dthetadt*sin(theta)), u_ - 0.5*m*l*pow(dthetadt,2)*sin(theta);


    T << M+m, 0.5*m*l*cos(theta), 0.5*m*l*cos(theta), I_p_f;

    Delta = (M+m)*(I_p_f) - pow(0.5*m*l*cos(theta),2);

    y_vec = 1/Delta*T*u_vec;

    ddtheta_dt2 = y_vec(0);
    ddx_dt2 = y_vec(1);


    // Pack state derivatives
    state_dot << dthetadt, ddtheta_dt2, dxdt, ddx_dt2;
}