#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <iostream>

class TerrainExplorer : public rclcpp::Node
{
public:
    TerrainExplorer() : Node("gps_navigator"), planet_radius_(6371000.0)
    {
        initializeDestinationCoordinates();
        initializeSubscribers();
        initializePublishers();
        RCLCPP_INFO(this->get_logger(), "TerrainExplorer initialized. Destination: (%.6f, %.6f)", dest_latitude_, dest_longitude_);
    }

private:
    const double planet_radius_;
    double dest_latitude_, dest_longitude_, current_orientation_;
    bool goal_reached_ = false;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    void initializeDestinationCoordinates()
    {
        std::cout << "Enter latitude: ";
        std::cin >> dest_latitude_;
        std::cout << "Enter longitude: ";
        std::cin >> dest_longitude_;
        this->declare_parameter("target_latitude", dest_latitude_);
        this->declare_parameter("target_longitude", dest_longitude_);
    }

    void initializeSubscribers()
    {
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&TerrainExplorer::gpsCallback, this, std::placeholders::_1));
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&TerrainExplorer::imuCallback, this, std::placeholders::_1));
    }

    void initializePublishers()
    {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (goal_reached_) return;
        auto [distance, bearing] = calculateNavigation(msg->latitude, msg->longitude);
        double heading_difference = normalizeAngle(bearing - current_orientation_);
        RCLCPP_INFO(this->get_logger(), "Navigation update - Distance: %.2f m, Bearing: %.2f rad, Heading diff: %.2f rad",
                    distance, bearing, heading_difference);
        if (distance < 0.5)
        {
            stopRover();
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Destination reached successfully!");
            return;
        }
        double angular_velocity = heading_difference;
        double linear_velocity = std::max(0.0, 1.0 - std::abs(heading_difference));
        moveRover(linear_velocity, angular_velocity);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_orientation_ = yaw;
    }

    std::pair<double, double> calculateNavigation(double lat, double lon)
    {
        double lat1 = toRadians(lat);
        double lon1 = toRadians(lon);
        double lat2 = toRadians(dest_latitude_);
        double lon2 = toRadians(dest_longitude_);
        double dlon = lon2 - lon1;
        double dlat = lat2 - lat1;
        double a = std::sin(dlat/2) * std::sin(dlat/2) +
                   std::cos(lat1) * std::cos(lat2) *
                   std::sin(dlon/2) * std::sin(dlon/2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
        double distance = planet_radius_ * c;
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

    void moveRover(double linear, double angular)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear;
        twist_msg->angular.z = angular;
        velocity_publisher_->publish(std::move(twist_msg));
    }

    void stopRover()
    {
        moveRover(0.0, 0.0);
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