#pragma once

#include <vector>
#include <sstream>

#include "ros/ros.h"
#include "fssim_messages/Command.h"
#include "fssim_messages/State.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"

#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "tf2/LinearMath/Quaternion.h"

class CarController {
    public:
        CarController(ros::NodeHandle &n);

    private:
        ros::Publisher _pub_command, _pub_vis;
        ros::Subscriber _sub_trajectory, _sub_state;

        fssim_messages::State _state;

        std::vector<geometry_msgs::Point32> _traj_coords;
        std::vector<double> _curv_map;

        double _min_curv, _max_curv;

        int _closest_traj_index;
        double _lookahead_dist;
        int _lookahead_index;
        int _speed_lookahead_index;

        double _target_speed;

        // Percentage along the x-axis for the piecewise cutoff.
        double _PIECEWISE_THRESHOLD;
        // How much to scale the downward gradient for the first half of the
        // piecewise function.
        double _PIECEWISE_M_FACTOR;

        double _LOOKAHEAD_FACTOR;
        double _MAX_SPEED;
        double _MIN_SPEED;

        /**
         * @brief Load controller parameters from config file.
         */
        void load_params(void);

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
        const double set_throttle(void);

        /**
         * @brief Set the steering value to be the pure pursuit curvature 
         * from the car position to the lookahead point. Steering is scaled
         * from between -1 and 1.
         */
        const double set_steering(void);

        /**
         * @brief Based on the curvature in front of the car, use a piecewise
         * linear mapping between curvature and target speed.
         */
        const double piecewise_linear(double curv);

        /**
         * @brief Looking in front of the car, find the worst 
         * case scenario, meaning the highest curvature.
         */
        const double get_max_lookahead_curv(void); 

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

        /***********************************************************************
        * Visualisation
        ***********************************************************************/

        enum Marker_ID {
            // Pure pursuit markers
            MARKER_POSITION,
            MARKER_CLOSEST_TRAJ,
            MARKER_LOOKAHEAD,
            MARKER_SPEED_LOOKAHEAD,

            // Other markers
            MARKER_HEADING,
            MARKER_CURVATURE,
            MARKER_STATE,
            MARKER_SPEED_SPLINE
        };

        /**
         * @brief Function to draw dots, mostly used in visualising pure pursuit
         * calculations.
         */
        void draw_pp_marker(double x, double y, std_msgs::ColorRGBA color_rbga, 
                int marker_id);

        /**
         * @brief Draw the direction the car is travelling in.
         */
        void draw_car_heading(void);

        /**
         * @brief Write text on screen, showing current state and control values.
         */
        void write_car_state(void);

        /**
         * @brief Draw a colour coded map of the track curvature.
         */
        void draw_curv_map(void);

        /**
         * @brief Draw a colour coded spline used to control the car speed.
         */
        void draw_speed_control_spline(void);
};