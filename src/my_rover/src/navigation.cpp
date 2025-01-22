#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>

class RoverNavigator : public rclcpp::Node
{
public:
    RoverNavigator()
        : Node("rover_navigator")
    {
        // Parameters for destination coordinates and stopping threshold
        this->declare_parameter("destination_lat", 0.00006);
        this->declare_parameter("destination_lon", 0.00004);
        this->declare_parameter("stopping_threshold", 0.5);

        this->get_parameter("destination_lat", destination_lat_);
        this->get_parameter("destination_lon", destination_lon_);
        this->get_parameter("stopping_threshold", stopping_threshold_);

        destination_reached_ = false;

        gps_data_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&RoverNavigator::processGpsData, this, std::placeholders::_1));

        imu_data_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&RoverNavigator::processImuData, this, std::placeholders::_1));

        velocity_command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Rover Navigator Node Initialized with Destination: Lat=%.6f, Lon=%.6f, Threshold=%.2f m", 
                    destination_lat_, destination_lon_, stopping_threshold_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_data_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_command_publisher_;

    double destination_lat_;
    double destination_lon_;
    double stopping_threshold_;
    std::optional<double> previous_lat_;
    std::optional<double> previous_lon_;
    double current_orientation_ = 0.0;
    bool destination_reached_;

    void processGpsData(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (destination_reached_)
        {
            publishVelocityCommand(0.0, 0.0);
            return;
        }

        double current_lat = msg->latitude;
        double current_lon = msg->longitude;

        if (previous_lat_ && previous_lon_)
        {
            current_orientation_ = calculateBearing(*previous_lat_, *previous_lon_, current_lat, current_lon);
        }

        previous_lat_ = current_lat;
        previous_lon_ = current_lon;

        double distance = computeHaversineDistance(current_lat, current_lon, destination_lat_, destination_lon_);
        double desired_orientation = calculateBearing(current_lat, current_lon, destination_lat_, destination_lon_);
        double orientation_error = adjustAngle(desired_orientation - current_orientation_);

        RCLCPP_INFO(this->get_logger(), "Distance to Destination: %.2f m", distance);
        RCLCPP_INFO(this->get_logger(), "Current Orientation: %.2f rad, Desired Orientation: %.2f rad, Orientation Error: %.2f rad",
                    current_orientation_, desired_orientation, orientation_error);

        if (distance < stopping_threshold_)
        {
            publishVelocityCommand(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Destination Reached.");
            destination_reached_ = true;
            return;
        }

        double angular_velocity = orientation_error * 2.0; // Adjusted gain for smoother turning
        double linear_velocity = std::max(0.0, 1.0 - std::abs(orientation_error)); // Scaled linear velocity

        publishVelocityCommand(linear_velocity, angular_velocity);
    }

    void processImuData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto q = msg->orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_orientation_ = adjustAngle(std::atan2(siny_cosp, cosy_cosp));

        RCLCPP_INFO(this->get_logger(), "Updated IMU Orientation: %.2f radians", current_orientation_);
    }

    double computeHaversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        const double R = 6371000;
        double dlat = degreesToRadians(lat2 - lat1);
        double dlon = degreesToRadians(lon2 - lon1);
        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(degreesToRadians(lat1)) * std::cos(degreesToRadians(lat2)) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        return R * c;
    }

    double calculateBearing(double lat1, double lon1, double lat2, double lon2)
    {
        double dlon = degreesToRadians(lon2 - lon1);
        double y = std::sin(dlon) * std::cos(degreesToRadians(lat2));
        double x = std::cos(degreesToRadians(lat1)) * std::sin(degreesToRadians(lat2)) -
                   std::sin(degreesToRadians(lat1)) * std::cos(degreesToRadians(lat2)) * std::cos(dlon);
        return adjustAngle(std::atan2(y, x));
    }

    double adjustAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    double degreesToRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    void publishVelocityCommand(double linear, double angular)
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear;
        twist_msg.angular.z = angular;
        velocity_command_publisher_->publish(twist_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverNavigator>());
    rclcpp::shutdown();
    return 0;
}
