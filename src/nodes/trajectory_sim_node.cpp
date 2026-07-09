// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.
//
// Trajectory simulation node: generates fake ground-truth motion
// (circle, figure-8, or dive patterns) and feeds it to the sensor
// drivers for demo purposes.
//
// This node simulates the submarine moving through a trajectory,
// computing the corresponding sensor readings (IMU, DVL, depth)
// and publishing them. The state estimator then fuses these to
// track the trajectory.
//
// Usage:
//   ros2 run submarine_drivers trajectory_sim_node
//   ros2 run submarine_drivers trajectory_sim_node --ros-args -p pattern:=figure8

#ifdef BUILD_ROS2_NODES

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <string>

namespace submarine_drivers {

class TrajectorySimNode : public rclcpp::Node {
public:
    TrajectorySimNode() : Node("trajectory_sim_node") {
        // Parameters
        this->declare_parameter("pattern", "circle");
        this->declare_parameter("speed", 2.0);        // m/s
        this->declare_parameter("radius", 10.0);       // meters
        this->declare_parameter("depth", 20.0);         // meters
        this->declare_parameter("dive_amplitude", 5.0); // meters
        this->declare_parameter("rate_hz", 50.0);

        pattern_ = this->get_parameter("pattern").as_string();
        speed_ = this->get_parameter("speed").as_double();
        radius_ = this->get_parameter("radius").as_double();
        depth_ = this->get_parameter("depth").as_double();
        dive_amplitude_ = this->get_parameter("dive_amplitude").as_double();
        double rate_hz = this->get_parameter("rate_hz").as_double();

        // Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/sensors/imu", 10);
        dvl_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/sensors/dvl", 10);
        depth_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/sensors/depth", 10);
        gt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ground_truth/pose", 10);
        gt_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/ground_truth/path", 10);

        gt_path_.header.frame_id = "odom";

        // TF broadcaster for RViz
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer
        double dt = 1.0 / rate_hz;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&TrajectorySimNode::update, this));

        sim_time_ = 0.0;
        dt_ = dt;

        RCLCPP_INFO(this->get_logger(),
            "Trajectory sim started: pattern=%s, speed=%.1f m/s, radius=%.1f m, depth=%.1f m",
            pattern_.c_str(), speed_, radius_, depth_);
    }

private:
    void update() {
        sim_time_ += dt_;

        // Compute ground-truth pose based on pattern
        double x, y, z, yaw, vx, vy, vz, yaw_rate;
        compute_trajectory(sim_time_, x, y, z, yaw, vx, vy, vz, yaw_rate);

        // Publish ground truth pose
        publish_ground_truth(x, y, z, yaw);

        // Broadcast TF: odom -> base_link
        broadcast_tf(x, y, z, yaw);

        // Publish simulated sensor readings
        publish_imu(vx, vy, yaw_rate);
        publish_dvl(vx, vy, vz);
        publish_depth(z);
    }

    void compute_trajectory(double t, double& x, double& y, double& z,
                            double& yaw, double& vx, double& vy,
                            double& vz, double& yaw_rate) {
        double omega = speed_ / radius_;  // angular velocity

        if (pattern_ == "circle") {
            x = radius_ * std::cos(omega * t);
            y = radius_ * std::sin(omega * t);
            z = depth_ + dive_amplitude_ * std::sin(0.1 * t);
            yaw = omega * t + M_PI / 2.0;  // tangent to circle

            vx = speed_;  // body-frame forward velocity
            vy = 0.0;
            vz = dive_amplitude_ * 0.1 * std::cos(0.1 * t);
            yaw_rate = omega;

        } else if (pattern_ == "figure8") {
            x = radius_ * std::sin(omega * t);
            y = radius_ * std::sin(2.0 * omega * t) / 2.0;
            z = depth_ + dive_amplitude_ * std::sin(0.2 * t);
            yaw = std::atan2(
                radius_ * 2.0 * omega * std::cos(2.0 * omega * t) / 2.0,
                radius_ * omega * std::cos(omega * t));

            double dx = radius_ * omega * std::cos(omega * t);
            double dy = radius_ * omega * std::cos(2.0 * omega * t);
            vx = std::sqrt(dx * dx + dy * dy);
            vy = 0.0;
            vz = dive_amplitude_ * 0.2 * std::cos(0.2 * t);
            yaw_rate = 0.0;  // simplified

        } else {
            // Straight line with gentle dive
            x = speed_ * t;
            y = 0.0;
            z = depth_ + dive_amplitude_ * std::sin(0.1 * t);
            yaw = 0.0;
            vx = speed_;
            vy = 0.0;
            vz = dive_amplitude_ * 0.1 * std::cos(0.1 * t);
            yaw_rate = 0.0;
        }
    }

    void publish_ground_truth(double x, double y, double z, double yaw) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;

        // Yaw-only quaternion
        msg.pose.orientation.w = std::cos(yaw / 2.0);
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = std::sin(yaw / 2.0);

        gt_pose_pub_->publish(msg);

        // Accumulate into path
        gt_path_.header.stamp = msg.header.stamp;
        gt_path_.poses.push_back(msg);
        // Keep path reasonable length
        if (gt_path_.poses.size() > 5000) {
            gt_path_.poses.erase(gt_path_.poses.begin());
        }
        gt_path_pub_->publish(gt_path_);
    }

    void broadcast_tf(double x, double y, double z, double yaw) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf.transform.rotation.w = std::cos(yaw / 2.0);
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = std::sin(yaw / 2.0);

        tf_broadcaster_->sendTransform(tf);
    }

    void publish_imu(double vx, double vy, double yaw_rate) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        // Centripetal acceleration for circular motion
        double omega = speed_ / radius_;
        msg.linear_acceleration.x = 0.0;  // constant speed, no surge accel
        msg.linear_acceleration.y = vx * omega;  // centripetal
        msg.linear_acceleration.z = 9.80665;  // gravity

        msg.angular_velocity.x = 0.0;
        msg.angular_velocity.y = 0.0;
        msg.angular_velocity.z = yaw_rate;

        // Add small noise for realism
        msg.linear_acceleration.x += 0.05 * (std::sin(sim_time_ * 100.0));
        msg.linear_acceleration.y += 0.05 * (std::cos(sim_time_ * 73.0));
        msg.angular_velocity.z += 0.002 * (std::sin(sim_time_ * 47.0));

        imu_pub_->publish(msg);
    }

    void publish_dvl(double vx, double vy, double vz) {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "dvl_link";
        msg.twist.linear.x = vx + 0.01 * std::sin(sim_time_ * 37.0);
        msg.twist.linear.y = vy + 0.01 * std::cos(sim_time_ * 53.0);
        msg.twist.linear.z = vz + 0.005 * std::sin(sim_time_ * 91.0);

        dvl_pub_->publish(msg);
    }

    void publish_depth(double z) {
        auto msg = std_msgs::msg::Float64();
        msg.data = z + 0.02 * std::sin(sim_time_ * 67.0);  // small noise
        depth_pub_->publish(msg);
    }

    // Parameters
    std::string pattern_;
    double speed_;
    double radius_;
    double depth_;
    double dive_amplitude_;
    double dt_;
    double sim_time_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gt_path_pub_;
    nav_msgs::msg::Path gt_path_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace submarine_drivers

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<submarine_drivers::TrajectorySimNode>());
    rclcpp::shutdown();
    return 0;
}

#endif  // BUILD_ROS2_NODES
