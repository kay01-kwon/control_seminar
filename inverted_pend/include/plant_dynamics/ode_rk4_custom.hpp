#ifndef ODE_RK4_CUSTOM_HPP
#define ODE_RK4_CUSTOM_HPP

#include <iostream>

template <typename StateInOut>
class OdeRk4Custom
{

    public:

    OdeRk4Custom() = default;

    ~OdeRk4Custom() = default;

    template <typename System>
    void do_step(System system_dynamics, StateInOut& state,
    const double &prev_time, const double &dt);

    private:

    double a1{1.0}, a2{2.0}, a3{2.0}, a4{1.0};

};

#endif // ODE_RK4_CUSTOM_HPP

template <typename StateInOut>
template <typename System>
inline void OdeRk4Custom<StateInOut>::do_step(System system_dynamics, 
StateInOut &s, const double &prev_time, 
const double &dt)
{
    StateInOut K1, K2, K3, K4;

    system_dynamics(s, K1, prev_time);

    StateInOut s_temp = s + 0.5*dt*K1;
    system_dynamics(s_temp, K2 ,prev_time + 0.5*dt);

    s_temp = s + 0.5*dt*K2;
    system_dynamics(s_temp, K3, prev_time + 0.5*dt);

    s_temp = s + dt*K3;
    system_dynamics(s_temp, K4, prev_time + dt);

    s += (K1*a1 + K2*a2 + K3*a3 + K4*a4)/static_cast<double>(6) * dt;
}