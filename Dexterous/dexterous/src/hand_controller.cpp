#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class HandControllerNode : public rclcpp::Node
{
public:
    HandControllerNode() : Node("hand_controller")
    {
        using namespace std::placeholders;

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // poseSub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        //     "usv/state/pose", 10,
        //     [this](const geometry_msgs::msg::Pose2D &msg)
        //     {
        //         this->pose.x = msg.x;
        //         this->pose.y = msg.y;
        //         // this->pose.theta = -msg.theta; // For SBG
        //         this->pose.theta = msg.theta; // For Simulations
        //     });

        // jointPub = this->create_publisher<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10);

        updateTimer =
            this->create_wall_timer(100ms, std::bind(&HandControllerNode::update, this));
    }


private:
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrivedPub;

    geometry_msgs::msg::Pose2D pose;

    rclcpp::TimerBase::SharedPtr updateTimer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void update() {
        std::string fromFrameRel = "base_link";
        std::string toFrameRel = "finger_1_1_1";

        geometry_msgs::msg::TransformStamped t;
        
        try {
            t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return;
        }
        RCLCPP_INFO(
        this->get_logger(), "Finger 1 EEF's Pose: %f, %f, %f",
        t.transform.translation.x, 
        t.transform.translation.y, 
        t.transform.translation.z
        );

        // headingDesPub->publish(heading_d);
        // wpErrorPub->publish(distance_e);
        // desiredPosePub->publish(wp);
        // arrivedPub->publish(this->arrived);
        // pivotPub->publish(pivot_set);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandControllerNode>());
    rclcpp::shutdown();
    return 0;
}
