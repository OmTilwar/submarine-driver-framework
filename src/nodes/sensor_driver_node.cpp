// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.
//
// ROS2 node wrapping the sensor drivers. Publishes sensor data on ROS2
// topics for consumption by the state estimator and other nodes.
//
// NOTE: This file requires ROS2 (rclcpp) to compile. Build with:
//   cmake -DBUILD_ROS2_NODES=ON ..
//
// Topics published:
//   /sensors/imu    (sensor_msgs/Imu)
//   /sensors/dvl    (geometry_msgs/TwistStamped)
//   /sensors/depth  (std_msgs/Float64)

#ifdef BUILD_ROS2_NODES

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "submarine_drivers/sensors/imu_driver.hpp"
#include "submarine_drivers/sensors/dvl_driver.hpp"
#include "submarine_drivers/sensors/depth_sensor_driver.hpp"

#include <memory>

namespace submarine_drivers {

class SensorDriverNode : public rclcpp::Node {
public:
    SensorDriverNode() : Node("sensor_driver_node") {
        // Initialize sensor drivers
        imu_ = std::make_shared<ImuDriver>();
        dvl_ = std::make_shared<DvlDriver>();
        depth_ = std::make_shared<DepthSensorDriver>();

        imu_->initialize();
        dvl_->initialize();
        depth_->initialize();

        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/sensors/imu", 10);
        dvl_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/sensors/dvl", 10);
        depth_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/sensors/depth", 10);

        // Create timer for sensor updates (200 Hz for IMU)
        imu_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&SensorDriverNode::imu_callback, this));

        // DVL at 5 Hz
        dvl_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&SensorDriverNode::dvl_callback, this));

        // Depth at 10 Hz
        depth_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorDriverNode::depth_callback, this));

        RCLCPP_INFO(this->get_logger(), "Sensor driver node initialized");
    }

    ~SensorDriverNode() override {
        imu_->shutdown();
        dvl_->shutdown();
        depth_->shutdown();
    }

private:
    void imu_callback() {
        imu_->update(0.005);
        if (!imu_->has_valid_reading()) return;

        auto reading = imu_->get_reading();
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        msg.linear_acceleration.x = reading.acceleration.x;
        msg.linear_acceleration.y = reading.acceleration.y;
        msg.linear_acceleration.z = reading.acceleration.z;
        msg.angular_velocity.x = reading.angular_velocity.x;
        msg.angular_velocity.y = reading.angular_velocity.y;
        msg.angular_velocity.z = reading.angular_velocity.z;
        imu_pub_->publish(msg);
    }

    void dvl_callback() {
        dvl_->update(0.2);
        if (!dvl_->has_valid_reading()) return;

        auto reading = dvl_->get_reading();
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "dvl_link";
        msg.twist.linear.x = reading.velocity.x;
        msg.twist.linear.y = reading.velocity.y;
        msg.twist.linear.z = reading.velocity.z;
        dvl_pub_->publish(msg);
    }

    void depth_callback() {
        depth_->update(0.1);
        if (!depth_->has_valid_reading()) return;

        auto reading = depth_->get_reading();
        auto msg = std_msgs::msg::Float64();
        msg.data = reading.depth;
        depth_pub_->publish(msg);
    }

    std::shared_ptr<ImuDriver> imu_;
    std::shared_ptr<DvlDriver> dvl_;
    std::shared_ptr<DepthSensorDriver> depth_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;

    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr dvl_timer_;
    rclcpp::TimerBase::SharedPtr depth_timer_;
};

}  // namespace submarine_drivers

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<submarine_drivers::SensorDriverNode>());
    rclcpp::shutdown();
    return 0;
}

#endif  // BUILD_ROS2_NODES
