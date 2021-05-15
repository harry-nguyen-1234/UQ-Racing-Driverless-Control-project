#include "car_controller.h"

void HSV_to_RGB(double &r, double &g, double &b, const double h) {
    // Formulas from https://github.com/python/cpython/blob/main/Lib/colorsys.py 
    auto s = 1.0;
    auto v = 1.0;
    auto i = int(h * 6.0);
    auto f = (h*6.0) - i;
    auto p = v*(1.0 - s);
    auto q = v*(1.0 - s*f);
    auto t = v*(1.0 - s*(1.0-f));
    i = i%6;

    if (i == 0) {
        r=v, g=t, b=p;
    } else if (i == 1) {
        r=q, g=v, b=p;
    } else if (i == 2) {
        r=p, g=v, b=t;
    } else if (i == 3) {
        r=p, g=q, b=v;
    } else if (i == 4) {
        r=t, g=p, b=v;
    } else if (i == 5) {
        r=v, g=p, b=q;
    }
}

void CarController::draw_pp_marker(double x, double y, 
        std_msgs::ColorRGBA color_rbga, int marker_id) {
    auto marker = visualization_msgs::Marker();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.color = color_rbga;

    _pub_vis.publish(marker);    
}

void CarController::draw_car_heading(void) {
    auto marker = visualization_msgs::Marker();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = MARKER_HEADING;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = _state.position.x;
    marker.pose.position.y = _state.position.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, _state.position.theta);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    auto max_arrow_length = 5.0;
    auto car_speed = std::hypot(_state.velocity.x, _state.velocity.y);
    auto arrow_length = car_speed / (_MAX_SPEED - _MIN_SPEED) * 
            max_arrow_length;
    marker.scale.x = arrow_length;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    std_msgs::ColorRGBA color_rgba;
    color_rgba.r = 1;
    color_rgba.g = 0;
    color_rgba.b = 0;
    color_rgba.a = 1;
    marker.color = color_rgba;

    _pub_vis.publish(marker);    
}

void CarController::write_car_state(void) {
    auto marker = visualization_msgs::Marker();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = MARKER_STATE;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = _state.position.x;
    marker.pose.position.y = _state.position.y;
    marker.pose.position.z = 2.5;

    marker.pose.orientation.w = 1;

    marker.scale.z = 0.5;

    std_msgs::ColorRGBA color_rgba;
    color_rgba.r = 1;
    color_rgba.g = 1;
    color_rgba.b = 1;
    color_rgba.a = 1;
    marker.color = color_rgba;

    auto car_speed = std::hypot(_state.velocity.x, _state.velocity.y);
    std::stringstream s;
    s.precision(2);
    s << std::fixed << "Current: " << car_speed << '\n' << 
            "Target: " << _target_speed << '\n' << 
            "Lookahead: " << _lookahead_dist << '\n' << 
            "Speed lookahead: " << _lookahead_dist * _LOOKAHEAD_FACTOR;
    marker.text = s.str();

    _pub_vis.publish(marker);    
}

void CarController::draw_curv_map(void) {
    auto HSV_MIN = 0.0;
    auto HSV_MAX = 240.0 / 360;

    auto marker = visualization_msgs::Marker();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = MARKER_CURVATURE;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.1;

    for (int i = 0; i < _curv_map.size(); i++) {
        auto color = _curv_map[i] / (_max_curv - _min_curv) * (HSV_MAX - 
                HSV_MIN);
        // Invert so the largest curvature maps to smallest HSV
        color = HSV_MAX - HSV_MIN - color; 

        double r, g, b;
        HSV_to_RGB(r, g, b, color);
        std_msgs::ColorRGBA color_rgba;
        color_rgba.r = r;
        color_rgba.g = g;
        color_rgba.b = b;
        color_rgba.a = 1;
        marker.colors.push_back(color_rgba);

        auto x = _traj_coords[i].x;
        auto y = _traj_coords[i].y;
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        marker.points.push_back(p);
    }

    _pub_vis.publish(marker);    
}

void CarController::draw_speed_control_spline(void) {
    auto HSV_MIN = 0.0;
    auto HSV_MAX = 240.0 / 360;

    auto marker = visualization_msgs::Marker();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = MARKER_SPEED_SPLINE;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.5;

    auto max_lookahead_curv = get_max_lookahead_curv();
    auto color = max_lookahead_curv / (_max_curv - _min_curv) * (HSV_MAX - 
            HSV_MIN);
    // Invert so the largest curvature maps to smallest HSV
    color = HSV_MAX - HSV_MIN - color; 

    double r, g, b;
    HSV_to_RGB(r, g, b, color);
    std_msgs::ColorRGBA color_rgba;
    color_rgba.r = r;
    color_rgba.g = g;
    color_rgba.b = b;
    color_rgba.a = 1;
    marker.color = color_rgba;

    auto i = _closest_traj_index;
    while (i != _speed_lookahead_index) {
        auto x = _traj_coords[i].x;
        auto y = _traj_coords[i].y;
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        marker.points.push_back(p);

        i = (i + 1) % _traj_coords.size();
    }

    _pub_vis.publish(marker);    
}