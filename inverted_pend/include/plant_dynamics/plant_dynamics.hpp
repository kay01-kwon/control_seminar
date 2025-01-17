#ifndef PLANT_DYNAMICS_HPP
#define PLANT_DYNAMICS_HPP
#include "state_def/state_def.hpp"

struct PlantParams
{
    double m;
    double M;
    double l;
};

class PlantDynamics
{
    public:

        PlantDynamics() = delete;

        PlantDynamics(const PlantParams &params);

        void set_input(const double &u);

        void get_state(State &state) const;
        
        void get_time(double &time) const;

        void dynamics(const State &state, 
        State &state_dot, const double &time);

    private:

        PlantParams params_;

        double gravity_ = 9.81;

        State state_{0, 0, 0, 0};

        double time_{0};

        double u_{0};

};



#endif // PLANT_DYNAMICS_HPP