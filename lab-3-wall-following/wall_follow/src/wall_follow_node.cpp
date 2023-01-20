#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class WallFollow : public rclcpp::Node
{

public:
    float angle_increment;
    float angle_min;
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
    }

private:
    // PID CONTROL PARAMS
    double kp = 1;
    double kd = 0;
    double ki = 0;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    double get_range(const float *range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
        int index = (angle - angle_min) / angle_increment;
        if (std::isnan(range_data[index]) || std::isinf(range_data[index]))
        {
            return 0;
        }
        else
        {
            return range_data[index];
        }
        // RCLCPP_INFO(this->get_logger(), "The ranges are : '%f'",range_data[0]);
    }

    double get_error(const float *range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        // int theta = 130;
        auto a = get_range(range_data, 60 * 3.14 / 180);
        auto b = get_range(range_data, 80 * 3.14 / 180);
        auto theta = a - b;
        auto alpha = atan((a * cos(theta) - b) / a * sin(theta));
        auto Dt = b * cos(alpha);
        Dt = dist - Dt;
        auto Dt1 = -(Dt + 0.9* sin(alpha));
        RCLCPP_INFO(this->get_logger(), "'%f'",Dt1);
        return Dt1;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */

        // TODO: Use kp, ki & kd to implement a PID controller
        rclcpp::Time prev_t(0, 0, RCL_ROS_TIME);
        rclcpp::Duration del_t = this->now() - prev_t;

        double del_t_nano = (double)del_t.nanoseconds();

        integral = prev_error * del_t_nano;
        double diff = (error - prev_error) / del_t_nano;

        double angle = kp * (error) + ki * (integral) + kd * (diff);
        // RCLCPP_INFO(this->get_logger(), "The ranges are : '%f %f'",angle,error);

        prev_error = error;
        prev_t = this->now();

        if (angle <= 10 && angle >= 0)
        {
            velocity = 1.0;
        }
        else if (angle <= 20 && angle > 10)
        {
            velocity = 0.5;
        }
        else
        {
            velocity = 1.5;
        }
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        pub_->publish(drive_msg);
        // TODO: fill in drive message and publish
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        angle_increment = scan_msg->angle_increment;
        angle_min = scan_msg->angle_min;
        double error = get_error((scan_msg->ranges.data()), 0.78); // TODO: replace with error calculated by get_error()
        double velocity = 0.5;                                     // TODO: calculate desired car velocity based on error
        // error = get_error(const_cast<float *>(scan_msg->ranges.data()), 3);
        // RCLCPP_INFO(this->get_logger(), "I received the error : '%f'", std::vector::size(scan_msg->ranges));
        // TODO: actuate the car with PID
        pid_control(error, velocity);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}