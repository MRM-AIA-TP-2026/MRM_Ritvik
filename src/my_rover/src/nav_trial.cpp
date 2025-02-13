#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>
#include <iostream>

class GpsNavigator : public rclcpp::Node
{
public:
    GpsNavigator()
        : Node("gps_navigator")
    {

        std::cout<<("Enter latitude: ");
        std::cin>>target_lat_;

        std::cout<<("Enter longitude: ");
        std::cin>>target_lon_;
        /*target_lat_ = 0.0002; // Example latitude
        target_lon_ = -0.00001; // Example longitude*/

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&GpsNavigator::gpsCallback, this, std::placeholders::_1));
        
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&GpsNavigator::imuCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    double target_lat_;
    double target_lon_;
    double current_heading_; // Heading from IMU
    bool target_reached_ = false;

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (target_reached_) {
            publishVelocity(0.0, 0.0);
            return;
        }

        double current_lat = msg->latitude;
        double current_lon = msg->longitude;

        // Calculate heading to target
        double distance = haversineDistance(current_lat, current_lon, target_lat_, target_lon_);
        double desired_heading = calculateHeading(current_lat, current_lon, target_lat_, target_lon_);

        // Normalize desired heading and current heading to [-π, π]
        desired_heading = normalizeAngle(desired_heading);
        current_heading_ = normalizeAngle(current_heading_);


        // Calculate angular error
       double angular_error = normalizeAngle((desired_heading-current_heading_));


        RCLCPP_INFO(this->get_logger(), "Distance to Target: %.2f meters", distance);
        RCLCPP_INFO(this->get_logger(), "Updated IMU Heading: %.2f radians", current_heading_);
        RCLCPP_INFO(this->get_logger(), "Current Heading: %.2f radians, Desired Heading: %.2f radians, Angular Error: %.2f radians",
                    current_heading_, desired_heading, angular_error);

        // Stop if close to the target
        if (distance < 0.5) // 1 meter threshold
        {
            publishVelocity(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Target reached.");
            target_reached_ = true;
            return;
        }

        // Calculate angular velocity and linear velocity
        double angular_velocity = angular_error; // Proportional control
       
        double linear_velocity = std::max(0.0, 1.0 - std::abs(angular_error)); // Reduce speed during turns
        
        publishVelocity(linear_velocity, angular_velocity);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract yaw from quaternion
        auto q = msg->orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_heading_ = std::atan2(siny_cosp, cosy_cosp);

    }

    double haversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        const double R = 6371000; // Earth's radius in meters
        const double to_rad = M_PI / 180.0;

        double dlat = (lat2 - lat1) * to_rad;
        double dlon = (lon2 - lon1) * to_rad;

        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1 * to_rad) * std::cos(lat2 * to_rad) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);

        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        return R * c;
    }

    double calculateHeading(double lat1, double lon1, double lat2, double lon2)
{
    const double to_rad = M_PI / 180.0;

    double dlon = (lon2 - lon1) * to_rad;

    double y = std::sin(dlon) * std::cos(lat2 * to_rad);
    double x = std::cos(lat1 * to_rad) * std::sin(lat2 * to_rad) -
               std::sin(lat1 * to_rad) * std::cos(lat2 * to_rad) * std::cos(dlon);

    double initial_heading = std::atan2(y, x);
    
    // Adjust to measure counterclockwise from east
   return M_PI+M_PI_2-initial_heading;
}
   /* double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = degreesToRadians(lat1);
    double lat2_rad = degreesToRadians(lat2);
    double dLon = degreesToRadians(lon2 - lon1);

    double y = sin(dLon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dLon);

    return atan2(y, x);  // Bearing in radians
}*/

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    void publishVelocity(double linear, double angular)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear;
        twist_msg.angular.z = angular;
        cmd_vel_publisher_->publish(twist_msg);
    }
};

int main(int argc, char *argv[])
{


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNavigator>());
    rclcpp::shutdown();
    return 0;
}


