#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>

using namespace std::chrono_literals;
double target_lat_, target_lon_, current_lat_, current_lon_,x,y,distance, bearing;

double total_bearing,total_distance,pew;
int n = 0;
int m = 0;
class GpsNavigator : public rclcpp::Node {
public:
    GpsNavigator() : Node("gps_navigator"){
        std::cout << "Input target latitude and longitude:" << std::endl;
        std::cin >> target_lat_;
        std::cin >> target_lon_;
        std::tie(distance, bearing) = calculateDistanceAndBearing(current_lat_, current_lon_, target_lat_, target_lon_);
        movement_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps", 10, std::bind(&GpsNavigator::gpsCallback, this, std::placeholders::_1));
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movement_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr inter_timer_;

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::tie(distance, bearing) = calculateDistanceAndBearing(current_lat_, current_lon_, target_lat_, target_lon_);
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        navigateToTarget();
    }
    
    void navigateToTarget() {
        if (bearing == 0)
        {
        end();
        }
        
        if (n == 0)
        {auto twist_msg = geometry_msgs::msg::Twist(); 
        n++;
        double time = std::abs(bearing/0.5)+0.5;
        auto inter = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(time));       
        twist_msg.angular.z = std::min(0.5,std::max(-1.0,bearing));
        twist_msg.linear.x = 0.0;
        movement_pub_->publish(twist_msg);
        inter_timer_ = this->create_wall_timer(inter, std::bind(&GpsNavigator::Straight, this));
        }    
}

void Straight()
{       auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        movement_pub_->publish(twist_msg);
        std::tie(distance, bearing) = calculateDistanceAndBearing(current_lat_, current_lon_, target_lat_, target_lon_);
        double t = std::abs(distance/0.5)+0.5;
        auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(t));
        twist_msg.linear.x = 0.5;
        twist_msg.angular.z = 0.0;
        movement_pub_->publish(twist_msg);
        timer_ = this->create_wall_timer(interval, std::bind(&GpsNavigator::end, this));
}

void end()
{auto twist_msg = geometry_msgs::msg::Twist();
twist_msg.linear.x = 0.0;
twist_msg.angular.z = 0.0;
movement_pub_->publish(twist_msg);
RCLCPP_INFO(this->get_logger(), "Target Reached\n");
rclcpp::shutdown();
exit(0);
}

    std::pair<double, double> calculateDistanceAndBearing(double lat1, double lon1, double lat2, double lon2) {
        const double deg_to_rad = M_PI / 180.0;
        lat1 *= deg_to_rad;
        lon1 *= deg_to_rad;
        lat2 *= deg_to_rad;
        lon2 *= deg_to_rad;

        const double dlat = lat2 - lat1;
        const double dlon = lon2 - lon1;

        const double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                         std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
        const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        double distance =  c;

        y = std::sin(dlon) * std::cos(lat2);
        x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
        double bearing = std::atan2(y, x);

        return {distance, bearing};
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNavigator>());
    rclcpp::shutdown();
    return 0;
}