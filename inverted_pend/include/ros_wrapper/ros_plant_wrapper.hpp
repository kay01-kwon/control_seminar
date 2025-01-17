#ifndef ROS_PLANT_WRAPPER_HPP
#define ROS_PLANT_WRAPPER_HPP
#include "plant_dynamics/plant_dynamics.hpp"
#include "plant_dynamics/ode_rk4_custom.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "inverted_pend/State_msg.h"
#include "inverted_pend/Unpause.h"

using std_msgs::Float64;
using ros::NodeHandle;
using inverted_pend::State_msg;
using inverted_pend::Unpause;

class RosPlantWrapper
{
    public:

        RosPlantWrapper(const NodeHandle &nh);

        ~RosPlantWrapper();

        bool unpause_callback(Unpause::Request &req, 
        Unpause::Response &res);

        void plant_input_callback(const std_msgs::Float64::ConstPtr &msg);

        void publish_state(const State& state);

        void run();

    private:

    ros::NodeHandle nh_;

    ros::ServiceServer plant_unpase_server_;

    ros::Publisher plant_state_pub_;
    ros::Subscriber plant_input_sub_;

    State state_;

    double time_now_, time_prev_, dt_;

    bool paused_{true};

    bool first_run_{true};

    OdeRk4Custom<State> *ode_solver_ptr;
    PlantDynamics *plant_dynamics_ptr;

};


#endif