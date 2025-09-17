#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <array>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/system.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavlink/common/mavlink.h>

// ---------------- Globals ----------------
using namespace mavsdk;
using std::placeholders::_1;
using namespace std::chrono_literals;

static double initial_roll  = -1.75;
static double initial_pitch = 1.58;
static double initial_yaw   = 1.68;

static int cam_orient = 1; // 1=downward, 0=forward
static auto start_time = std::chrono::steady_clock::now();

std::shared_ptr<MavlinkPassthrough> passthrough;

// ---------------- Math Helpers ----------------
double normalize_angle(double angle) {
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

double normalize_roll(double current_roll, double ref_roll) {
    return normalize_angle(-(current_roll - ref_roll));
}

double normalize_pitch(double current_pitch, double ref_pitch) {
    return normalize_angle(-(current_pitch + ref_pitch));
}

double get_relative_yaw(double yaw) {
    return yaw - initial_yaw;
}

double get_relative_roll(double roll) {
    return normalize_roll(roll, initial_roll);
}

double get_relative_pitch(double pitch) {
    return normalize_pitch(pitch, initial_pitch);
}

std::array<double,3> rotate_to_world(const std::array<double,3> &attitude) {
    double roll = attitude[0];
    double pitch = attitude[1];
    double yaw = attitude[2];

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    double R[3][3] = {
        { cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy },
        { cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy },
        { -sp,   sr*cp,             cr*cp }
    };

    return { atan2(R[2][1], R[2][2]),
             asin(-R[2][0]),
             atan2(R[1][0], R[0][0]) };
}

// ---------------- MAVSDK Send Helpers ----------------
void vision_position_send(double x, double y, double z, double roll, double pitch, double yaw) {
    mavlink_message_t msg;

    float cov[21];
    for (int i = 0; i < 21; i++) cov[i] = NAN;  // no covariance
    uint8_t reset_counter = 0;

    mavlink_msg_vision_position_estimate_pack(
        1, 200, &msg,
        static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000),
        x, y, z, roll, pitch, yaw,
        cov, reset_counter
    );

    passthrough->send_message(msg);  // MAVSDK v1 API
}

void vision_speed_send(double vx, double vy, double vz) {
    mavlink_message_t msg;

    float cov[9];
    for (int i = 0; i < 9; i++) cov[i] = NAN;  // no covariance
    uint8_t reset_counter = 0;

    mavlink_msg_vision_speed_estimate_pack(
        1, 200, &msg,
        static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000),
        vx, vy, vz,
        cov, reset_counter
    );

    passthrough->send_message(msg);  // MAVSDK v1 API
}

// ---------------- ROS2 Node ----------------
class SlamLocalization : public rclcpp::Node {
public:
    SlamLocalization() : Node("slam_localization"), counter_(0) {
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odomimu", qos,
            std::bind(&SlamLocalization::odom_callback, this, _1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto pos = msg->pose.pose.position;
        auto vel = msg->twist.twist.linear;
        auto ori = msg->pose.pose.orientation;

        // Quaternion -> Euler
        double sinr_cosp = 2 * (ori.w * ori.x + ori.y * ori.z);
        double cosr_cosp = 1 - 2 * (ori.x * ori.x + ori.y * ori.y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (ori.w * ori.y - ori.z * ori.x);
        double pitch = (std::abs(sinp) >= 1) ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);

        double siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y);
        double cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        if (cam_orient == 0) {
            double cam_x = pos.x, cam_y = -pos.y, cam_z = -pos.z;
            double cam_vx = vel.x, cam_vy = -vel.y, cam_vz = -vel.z;
            double cam_roll = -pitch;
            double cam_pitch = get_relative_pitch(yaw);
            double cam_yaw = get_relative_yaw(-roll);

            RCLCPP_INFO(this->get_logger(), "[Forward] roll: %.2f pitch: %.2f yaw: %.2f",
                        cam_roll, cam_pitch, cam_yaw);
            vision_speed_send(cam_vx, cam_vy, cam_vz);
            vision_position_send(cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw);

        } else if (cam_orient == 1) {
            std::array<double,3> att = {roll, pitch, yaw};
            auto world = rotate_to_world(att);

            double cam_x = pos.z, cam_y = -pos.y, cam_z = pos.x;
            double cam_roll = get_relative_roll(world[0]);
            double cam_pitch = get_relative_pitch(world[2]);
            double cam_yaw = -world[1];
            double cam_vx = vel.z, cam_vy = -vel.y, cam_vz = vel.x;

            RCLCPP_INFO(this->get_logger(), "[Downward] roll: %.2f pitch: %.2f yaw: %.2f",
                        cam_roll, cam_pitch, cam_yaw);
            vision_speed_send(cam_vx, cam_vy, cam_vz);
            vision_position_send(cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw);
        }

        counter_++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        double hz = counter_ / elapsed;
        RCLCPP_INFO(this->get_logger(), "Sending to FCU %.2f Hz", hz);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    int counter_;
};

// ---------------- Main ----------------
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // ✅ Use working constructor with explicit configuration
    Mavsdk::Configuration config{ComponentType::GroundStation};
    Mavsdk mavsdk(config);

    std::string conn_url = "serial:///dev/ttyACM0:115200";
    ConnectionResult connection_result = mavsdk.add_any_connection(conn_url);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    std::shared_ptr<System> system;
    for (int i = 0; i < 10; ++i) {
        if (!mavsdk.systems().empty()) {
            system = mavsdk.systems().at(0);
            break;
        }
        std::this_thread::sleep_for(1s);
    }
    if (!system) {
        std::cerr << "No system found." << std::endl;
        return 1;
    }

    passthrough = std::make_shared<MavlinkPassthrough>(system);

    auto node = std::make_shared<SlamLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
