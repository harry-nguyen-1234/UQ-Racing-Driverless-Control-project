#include "car_controller.h"

CarController::CarController(ros::NodeHandle &n) {

    _pub_command = n.advertise<fssim_messages::Command>("chicken/cmd", 1, true);

    _sub_trajectory = n.subscribe("chicken/trajectory", 1, 
            &CarController::callback_trajectory, this);

    _sub_state = n.subscribe("chicken/state", 1, &CarController::callback_state, 
            this);
}

void CarController::pure_pursuit(void) {
    update_closest_traj_index();
    update_lookahead_dist();
    update_lookahead_index();
    update_speed_lookahead_index();
    update_target_speed();
    drive();
}

void CarController::update_closest_traj_index(void) {
    const auto car_x = _state.position.x;
    const auto car_y = _state.position.y;

    std::vector<float> distances;
    for (const auto &p : _traj_coords) {
        const auto distance = std::hypot(p.x - car_x, p.y - car_y);
        distances.push_back(distance);
    }

    const auto it = std::min_element(distances.begin(), 
            distances.end());
    const auto min_ele_index = std::distance(distances.begin(), it);

    _closest_traj_index = min_ele_index;
}

void CarController::update_lookahead_dist(void) {
    const auto car_speed = std::hypot(_state.velocity.x, _state.velocity.y);

    const auto min_lookahead_dist = 1.0;
    const auto min_track_radius = 1.0 / _max_curv;
    const auto max_lookahead_dist = min_track_radius;

    _lookahead_dist = car_speed / (_MAX_SPEED - _MIN_SPEED) * 
            (max_lookahead_dist - min_lookahead_dist);

    if (_lookahead_dist < min_lookahead_dist) {
        _lookahead_dist = min_lookahead_dist;
    } else if (_lookahead_dist > max_lookahead_dist) {
        _lookahead_dist = max_lookahead_dist;
    }
}

void CarController::update_lookahead_index(void) {
    _lookahead_index = (_closest_traj_index + 1) % _traj_coords.size();
    const auto closest_traj_x = _traj_coords[_closest_traj_index].x;
    const auto closest_traj_y = _traj_coords[_closest_traj_index].y;

    while (true) {
        const auto lookahead_x = _traj_coords[_lookahead_index].x;
        const auto lookahead_y = _traj_coords[_lookahead_index].y;
        const auto chord_length = std::hypot(closest_traj_x - lookahead_x, 
                closest_traj_y - lookahead_y);

        if (chord_length > _lookahead_dist) {
            break;
        }
        _lookahead_index = (_lookahead_index + 1) % _traj_coords.size();
    }
}

void CarController::update_speed_lookahead_index(void) {
    _speed_lookahead_index = (_closest_traj_index + 1) % _traj_coords.size();
    const auto closest_traj_x = _traj_coords[_closest_traj_index].x;
    const auto closest_traj_y = _traj_coords[_closest_traj_index].y;

    while (true) {
        const auto speed_lookahead_x = _traj_coords[_speed_lookahead_index].x;
        const auto speed_lookahead_y = _traj_coords[_speed_lookahead_index].y;
        const auto chord_length = std::hypot(closest_traj_x - speed_lookahead_x, 
                closest_traj_y - speed_lookahead_y);

        if (chord_length > _lookahead_dist * _LOOKAHEAD_FACTOR) {
            break;
        }
        _speed_lookahead_index = (_speed_lookahead_index + 1) % 
                _traj_coords.size();
    }
}

void CarController::drive(void) {
    const auto throttle_cmd = set_throttle();
    const auto steering_cmd = set_steering();

    fssim_messages::Command cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "map";
    cmd.throttle = throttle_cmd;
    cmd.steering_angle = steering_cmd;

    _pub_command.publish(cmd);
}

const float CarController::set_throttle(void) {
    const auto car_speed = std::hypot(_state.velocity.x, _state.velocity.y);
    auto throttle = (_target_speed - car_speed) / _target_speed;

    if (throttle < -1) {
        throttle = -1;
    } else if (throttle > 1) {
        throttle = 1;
    }

    return throttle;
}

void CarController::update_target_speed(void) { 
    const auto max_lookahead_curv = get_max_lookahead_curv();
    _target_speed = piecewise_linear(max_lookahead_curv);

    if (_target_speed < _MIN_SPEED) {
        _target_speed = _MIN_SPEED;
    } else if (_target_speed > _MAX_SPEED) {
        _target_speed = _MAX_SPEED;
    }
}

const float CarController::piecewise_linear(float curv) {
    const auto m = (_MAX_SPEED - _MIN_SPEED) / (_min_curv - _max_curv);
    const auto x_stop = (_max_curv - _min_curv) * _PIECEWISE_THRESHOLD + 
            _min_curv;

    const auto m1 = _PIECEWISE_M_FACTOR * m;
    const auto c1 = _MAX_SPEED;

    const auto y_stop = m1 * x_stop + c1;
    const auto m2 = (y_stop - _MIN_SPEED) / (x_stop - _max_curv);
    const auto c2 = y_stop - m2 * x_stop;

    float speed;
    if (curv < x_stop) {
        speed = m1 * curv + c1;
    } else {
        speed = m2 * curv + c2;
    }

    return speed;
}

const float CarController::get_max_lookahead_curv(void) {
    std::vector<float> lookahead_curvs;

    auto i = _closest_traj_index;
    while (i != _speed_lookahead_index) {
        lookahead_curvs.push_back(_curv_map[i]);
        i = (i + 1) % _curv_map.size();
    }

    const auto max_lookahead_curv = *std::max_element(lookahead_curvs.begin(), 
            lookahead_curvs.end());

    return max_lookahead_curv;
}

const float CarController::set_steering(void) {
    const auto lookahead_x = _traj_coords[_lookahead_index].x;
    const auto lookahead_y = _traj_coords[_lookahead_index].y;

    const auto x_offset = _state.position.x - lookahead_x;
    const auto y_offset = _state.position.y - lookahead_y;
    const auto theta = std::atan2(y_offset, x_offset);
    const auto car_heading = _state.position.theta;
    const auto angle_offset = car_heading - theta;

    const auto dist_to_point = std::hypot(x_offset, y_offset);
    const auto curvature = 2.0 * std::sin(angle_offset) / dist_to_point;

    auto steering = curvature;
    if (steering > 1.0) {
        steering = 1.0;
    } else if (steering < -1.0) {
        steering = -1.0;    
    }

    return steering;
}

void CarController::update_trajectory(
        const geometry_msgs::PolygonStamped &traj) {
    _traj_coords.clear();
    _traj_coords = traj.polygon.points;
}

void CarController::update_curv_map(void) {
    _curv_map.clear();

    // Set the search interval to be 1% the number of trajectory coordinates.
    // This assumes the trajectory has a reasonably high resolution.
    const int num_points = _traj_coords.size();
    const int search_interval = std::ceil(0.01 * num_points);
    for (int index_this = 0; index_this < num_points; index_this++) {
        const auto index_next = (index_this + search_interval) % num_points;
        const auto index_prev = (index_this + num_points - search_interval) % 
                num_points;
        const auto p_a = _traj_coords[index_this];
        const auto p_b = _traj_coords[index_next];
        const auto p_c = _traj_coords[index_prev];

        const auto dist_AB = std::hypot(p_a.x - p_b.x, p_a.y - p_b.y);
        const auto dist_AC = std::hypot(p_a.x - p_c.x, p_a.y - p_c.y);
        const auto dist_BC = std::hypot(p_b.x - p_c.x, p_b.y - p_c.y);

        std::vector<float> lengths = {dist_AB, dist_AC, dist_BC};
        // Sort lengths in descending order for numerically stable 
        // Heron's formula.
        std::sort(lengths.begin(), lengths.end(), std::greater<float>());
        const auto a = lengths[0];
        const auto b = lengths[1];
        const auto c = lengths[2];

        const auto denominator = a * b * c;
        float curvature;
        if (denominator == 0) {
            curvature = 0;
        } else {
            // Calculate triangle area using Heron's formula.
            const auto area = 1.0/4 * 
                    std::sqrt((a+(b+c))*(c-(a-b))*(c+(a-b))*(a+(b-c)));
            curvature = 4.0 * area / denominator;
        }
        _curv_map.push_back(curvature);
    }

    _min_curv = *std::min_element(_curv_map.begin(), _curv_map.end());
    _max_curv = *std::max_element(_curv_map.begin(), _curv_map.end());
}

void CarController::callback_trajectory(
        const geometry_msgs::PolygonStamped &traj) {
    update_trajectory(traj);
    update_curv_map();
}

void CarController::callback_state(const fssim_messages::State &s) {
    _state = s;

    if (!_traj_coords.empty()) {
        pure_pursuit();
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "car_controller");
    ros::NodeHandle n;
    auto car_controller = CarController(n);

    ros::spin();

    return 0;
}