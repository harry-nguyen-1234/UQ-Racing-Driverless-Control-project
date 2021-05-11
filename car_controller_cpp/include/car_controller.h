#pragma once

#include <vector>

#include "ros/ros.h"
#include "fssim_messages/Command.h"
#include "fssim_messages/State.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"

class CarController {
    public:
        CarController(ros::NodeHandle &n);

    private:

        ros::Publisher _pub_command;
        ros::Subscriber _sub_trajectory, _sub_state;

        fssim_messages::State _state;

        std::vector<geometry_msgs::Point32> _traj_coords;
        std::vector<float> _curv_map;

        float _min_curv, _max_curv;

        int _closest_traj_index;
        float _lookahead_dist;
        int _lookahead_index;
        int _speed_lookahead_index;

        float _target_speed;

        // Percentage along the x-axis for the piecewise cutoff.
        const float _PIECEWISE_THRESHOLD = 0.2;
        // How much to scale the downward gradient for the first half of the
        // piecewise function.
        const float _PIECEWISE_M_FACTOR = 3.5;

        const float _LOOKAHEAD_FACTOR = 5.0;
        const float _MAX_SPEED = 25.0;
        const float _MIN_SPEED = 5.0;

        /**
         * @brief Perform the pure pursuit pipeline.
         */
        void pure_pursuit(void);

        /**
         * @brief Get the index of the closest trajecory point.
         */
        void update_closest_traj_index(void);

        /**
         * @brief Based on how fast the car is going, get the lookahead 
         */
        void update_lookahead_dist(void);

        /**
         * @brief Get the index of the lookahead point that is the lookahead
         * distance away from the closest trajectry point.
         */
        void update_lookahead_index(void);

        /**
         * @brief Get the index of the lookahead point used to control speed,
         * separate from the steering lookahead point. The speed lookahead
         * point is rather further away than the steering control.
         */
        void update_speed_lookahead_index(void);

        /**
         * @brief Based on the curvature in front of the car, change the 
         * target speed.
         */
        void update_target_speed(void); 

        /**
         * @brief Send throttle and steering commands.
         */
        void drive(void);

        /**
         * @brief Based on the difference between the target speed and current
         * car speed, set the throttle. Throttle is scaled between -1 and 1.
         */
        const float set_throttle(void);

        /**
         * @brief Set the steering value to be the pure pursuit curvature 
         * from the car position to the lookahead point. Steering is scaled
         * from between -1 and 1.
         */
        const float set_steering(void);

        /**
         * @brief Based on the curvature in front of the car, use a piecewise
         * linear mapping between curvature and target speed.
         */
        const float piecewise_linear(float curv);

        /**
         * @brief Looking in front of the car, find the worst 
         * case scenario, meaning the highest curvature.
         */
        const float get_max_lookahead_curv(void); 

        /**
         * @brief When called from the subscriber, update the stored trajectory.
         */
        void update_trajectory(const geometry_msgs::PolygonStamped &traj);

        /**
         * @brief For a given trajectory, calculate and store its curvature.
         * Calculate curvature using circumradii of three iteratively sampled
         * points.
         */
        void update_curv_map(void);

        void callback_trajectory(const geometry_msgs::PolygonStamped &traj);
        void callback_state(const fssim_messages::State &s);
};