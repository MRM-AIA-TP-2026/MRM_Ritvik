#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <memory>
#include <iostream>

class TerrainExplorer : public rclcpp::Node
{
public:
    TerrainExplorer() : Node("gps_navigator"), earth_radius_(6371000.0)
    {
        initializeTargetCoordinates();
        initializeSubscribers();
        initializePublishers();
        RCLCPP_INFO(this->get_logger(), "TerrainExplorer initialized. Target: (%.6f, %.6f)", target_lat_, target_lon_);
    }

private:
    const double earth_radius_;
    double target_lat_, target_lon_, current_yaw_;
    bool mission_complete_ = false;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motion_pub_;

    void initializeTargetCoordinates()
    {
        std::cout << "Enter latitude: ";
        std::cin >> target_lat_;

        std::cout << "Enter longitude: ";
        std::cin >> target_lon_;

        this->declare_parameter("target_latitude", target_lat_);
        this->declare_parameter("target_longitude", target_lon_);
    }

    void initializeSubscribers()
    {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&TerrainExplorer::gpsDataHandler, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&TerrainExplorer::imuDataHandler, this, std::placeholders::_1));
    }

    void initializePublishers()
    {
        motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    void gpsDataHandler(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (mission_complete_) return;

        auto [distance, bearing] = computeNavigationData(msg->latitude, msg->longitude);
        double heading_diff = normalizeAngle(bearing - current_yaw_);

        RCLCPP_INFO(this->get_logger(), "Navigation update - Distance: %.2f m, Bearing: %.2f rad, Heading diff: %.2f rad",
                    distance, bearing, heading_diff);

        if (distance < 0.5)
        {
            haltRover();
            mission_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "Destination reached successfully!");
            return;
        }

        double angular_speed = heading_diff;
        double linear_speed = std::max(0.0, 1.0 - std::abs(heading_diff));
        
        commandRoverMotion(linear_speed, angular_speed);
    }

    void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
    }

    std::pair<double, double> computeNavigationData(double lat, double lon)
    {
        double lat1 = toRadians(lat);
        double lon1 = toRadians(lon);
        double lat2 = toRadians(target_lat_);
        double lon2 = toRadians(target_lon_);

        double dlon = lon2 - lon1;
        double dlat = lat2 - lat1;

        double a = std::sin(dlat/2) * std::sin(dlat/2) +
                   std::cos(lat1) * std::cos(lat2) *
                   std::sin(dlon/2) * std::sin(dlon/2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
        double distance = earth_radius_ * c;

        double y = std::sin(dlon) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) -
                   std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
        double bearing = std::atan2(y, x);
        bearing = M_PI + M_PI_2 - bearing;

        return {distance, bearing};
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    void commandRoverMotion(double linear, double angular)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear;
        twist_msg->angular.z = angular;
        motion_pub_->publish(std::move(twist_msg));
    }

    void haltRover()
    {
        commandRoverMotion(0.0, 0.0);
    }

    double toRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainExplorer>());
    rclcpp::shutdown();
    return 0;
}
