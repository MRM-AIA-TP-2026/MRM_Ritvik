#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <iostream>

class RoverNavigator : public rclcpp::Node {
public:
    RoverNavigator() : Node("gps_navigator"), heading_(0.0), reached_target_(false) 
 {
        std::cout << "Enter target latitude: ";
        std::cin >> target_latitude_;
        
        std::cout << "Enter target longitude: ";
        std::cin >> target_longitude_;
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&RoverNavigator::onGpsUpdate, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&RoverNavigator::onImuUpdate, this, std::placeholders::_1));

        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    
    double target_latitude_;
    double target_longitude_;
    double heading_;
    bool reached_target_;

    void onGpsUpdate(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (reached_target_) {
            issueMovementCommand(0.0, 0.0);
            return;
        }

        double current_lat = msg->latitude;
        double current_lon = msg->longitude;

        double distance_to_goal = computeHaversine(current_lat, current_lon, target_latitude_, target_longitude_);
        double required_heading = computeTargetHeading(current_lat, current_lon, target_latitude_, target_longitude_);
        
        double heading_error = normalizeAngle(required_heading - heading_);

        RCLCPP_INFO(this->get_logger(), "Distance: %.2f m | Current Heading: %.2f rad | Target Heading: %.2f rad | Heading Error: %.2f rad", 
                    distance_to_goal, heading_, required_heading, heading_error);

        if (distance_to_goal < 0.5) {
            issueMovementCommand(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Target reached.");
            reached_target_ = true;
            return;
        }

        double angular_velocity = heading_error;
        double linear_velocity = std::max(0.0, 1.0 - std::abs(heading_error));
        
        issueMovementCommand(linear_velocity, angular_velocity);
    }

    void onImuUpdate(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto q = msg->orientation;
        double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
        double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        heading_ = std::atan2(sin_yaw, cos_yaw);
    }

    double computeHaversine(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000;
        const double to_rad = M_PI / 180.0;
        
        double dlat = (lat2 - lat1) * to_rad;
        double dlon = (lon2 - lon1) * to_rad;
        
        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1 * to_rad) * std::cos(lat2 * to_rad) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
        
        return R * (2 * std::atan2(std::sqrt(a), std::sqrt(1 - a)));
    }

    double computeTargetHeading(double lat1, double lon1, double lat2, double lon2) {
        const double to_rad = M_PI / 180.0;
        double delta_lon = (lon2 - lon1) * to_rad;
        
        double y = std::sin(delta_lon) * std::cos(lat2 * to_rad);
        double x = std::cos(lat1 * to_rad) * std::sin(lat2 * to_rad) -
                   std::sin(lat1 * to_rad) * std::cos(lat2 * to_rad) * std::cos(delta_lon);
        
        return normalizeAngle(M_PI + M_PI_2 - std::atan2(y, x));
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    void issueMovementCommand(double linear, double angular) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear;
        msg.angular.z = angular;
        velocity_pub_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverNavigator>());
    rclcpp::shutdown();
    return 0;
}

