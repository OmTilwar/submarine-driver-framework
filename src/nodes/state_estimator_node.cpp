// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.
//
// ROS2 node for the 6-DOF state estimator. Subscribes to sensor topics
// and publishes the fused pose estimate.
//
// NOTE: This file requires ROS2 (rclcpp) to compile. Build with:
//   cmake -DBUILD_ROS2_NODES=ON ..
//
// Subscriptions:
//   /sensors/imu    (sensor_msgs/Imu)       - prediction at 200 Hz
//   /sensors/dvl    (geometry_msgs/TwistStamped) - velocity update at 5 Hz
//   /sensors/depth  (std_msgs/Float64)      - depth update at 10 Hz
//
// Publications:
//   /estimation/pose (geometry_msgs/PoseStamped) - fused 6-DOF pose

#ifdef BUILD_ROS2_NODES

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "submarine_drivers/estimation/state_estimator_6dof.hpp"
#include "submarine_drivers/core/types.hpp"

#include <memory>

namespace submarine_drivers {

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode() : Node("state_estimator_node") {
        estimator_ = std::make_unique<StateEstimator6DOF>();
        last_imu_time_ = this->now();

        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/estimation/pose", 10);

        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu", 10,
            std::bind(&StateEstimatorNode::imu_callback, this,
                      std::placeholders::_1));

        dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/sensors/dvl", 10,
            std::bind(&StateEstimatorNode::dvl_callback, this,
                      std::placeholders::_1));

        depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sensors/depth", 10,
            std::bind(&StateEstimatorNode::depth_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "State estimator node initialized");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto current_time = this->now();
        double dt = (current_time - last_imu_time_).seconds();
        last_imu_time_ = current_time;

        if (dt <= 0.0 || dt > 1.0) return;

        ImuReading imu;
        imu.acceleration = Vector3(msg->linear_acceleration.x,
                                    msg->linear_acceleration.y,
                                    msg->linear_acceleration.z);
        imu.angular_velocity = Vector3(msg->angular_velocity.x,
                                        msg->angular_velocity.y,
                                        msg->angular_velocity.z);
        imu.valid = true;

        estimator_->predict(imu, dt);
        publish_pose();
    }

    void dvl_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        DvlReading dvl;
        dvl.velocity = Vector3(msg->twist.linear.x,
                                msg->twist.linear.y,
                                msg->twist.linear.z);
        dvl.bottom_lock = true;
        dvl.valid = true;

        estimator_->update_dvl(dvl);
    }

    void depth_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        DepthReading depth;
        depth.depth = msg->data;
        depth.valid = true;

        estimator_->update_depth(depth);
    }

    void publish_pose() {
        auto pose = estimator_->get_pose();
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.pose.position.x = pose.position.x;
        msg.pose.position.y = pose.position.y;
        msg.pose.position.z = pose.position.z;

        // Convert Euler to quaternion (simplified)
        double cr = std::cos(pose.orientation.roll * 0.5);
        double sr = std::sin(pose.orientation.roll * 0.5);
        double cp = std::cos(pose.orientation.pitch * 0.5);
        double sp = std::sin(pose.orientation.pitch * 0.5);
        double cy = std::cos(pose.orientation.yaw * 0.5);
        double sy = std::sin(pose.orientation.yaw * 0.5);

        msg.pose.orientation.w = cr * cp * cy + sr * sp * sy;
        msg.pose.orientation.x = sr * cp * cy - cr * sp * sy;
        msg.pose.orientation.y = cr * sp * cy + sr * cp * sy;
        msg.pose.orientation.z = cr * cp * sy - sr * sp * cy;

        pose_pub_->publish(msg);
    }

    std::unique_ptr<StateEstimator6DOF> estimator_;
    rclcpp::Time last_imu_time_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
};

}  // namespace submarine_drivers

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<submarine_drivers::StateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}

#endif  // BUILD_ROS2_NODES
