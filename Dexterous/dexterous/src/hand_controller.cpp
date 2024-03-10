#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

        eef1_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/eef1_marker", 10);
        eef1_estimated_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/eef1_estimated_marker", 10);

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState &msg)
            {
                this->j_s = msg;
            });

        // jointPub = this->create_publisher<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10);

        updateTimer =
            this->create_wall_timer(100ms, std::bind(&HandControllerNode::update, this));
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr eef1_marker_pub_, eef1_estimated_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrivedPub;

    geometry_msgs::msg::Pose2D estimated_pose;
    visualization_msgs::msg::Marker eef1_marker, eef1_estimated_marker;
    sensor_msgs::msg::JointState j_s;

    rclcpp::TimerBase::SharedPtr updateTimer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    float L1{0}, L2{0}, L3{0};
    float q1{0}, q2{0}, q3{0};

    Eigen::Vector3f f1_calc, f1_rotated;
    Eigen::Matrix3f f1_rot_mat;

    float f1_calculated[3];
    float f2_calculated[3];
    float f3_calculated[3];

    float f1_estimated[3];
    float f2_estimated[3];
    float f3_estimated[3];

    float base_2_f1[3] = {0.308639953146409, 0.175, 1.22256279986678};

    float f1_rotation[3] = {3.14159265358979, -0.582486579623969, 0};

    Eigen::Vector3f get_rotated(Eigen::Vector3f v, float rot[3]){
        Eigen::Matrix3f rot_x, rot_y, rot_z, rot_mat;
        rot_x << 
            1, 0, 0,
            0, cos(rot[0]), sin(rot[0]),
            0, -sin(rot[0]), cos(rot[0]);
        rot_y << 
            cos(rot[1]), 0, -sin(rot[1]),
            0, 1, 0,
            sin(rot[1]), 0, cos(rot[1]);
        rot_z << 
            cos(rot[2]), sin(rot[2]), 0,
            -sin(rot[2]), cos(rot[2]), 0,
            0, 0, 1;
        return rot_x * (rot_y * (rot_z * v));
    }
    
    void update() {
        std::string finger1_frame = "finger_1_eef";
        std::string toFrameRel = "base_link";

        geometry_msgs::msg::TransformStamped t;
        
        // try {
            t = tf_buffer_->lookupTransform(
            toFrameRel, finger1_frame,
            tf2::TimePointZero);
        // } catch (const tf2::TransformException & ex) {
        //     RCLCPP_INFO(
        //     this->get_logger(), "Could not transform %s to %s: %s",
        //     toFrameRel.c_str(), finger1_frame.c_str(), ex.what());
        //     return;
        // }
        RCLCPP_INFO(
        this->get_logger(), "Finger 1 EEF's Pose: %f, %f, %f",
        t.transform.translation.x, 
        t.transform.translation.y, 
        t.transform.translation.z
        );

        RCLCPP_INFO(
        this->get_logger(), "Finger 1 Joint States: %f, %f, %f",
        j_s.position[0], 
        j_s.position[1], 
        j_s.position[2]
        );

        /*
        FINGER 1 MEASUREMENTS:
        Base_link -> L1 = 1.273 = x: 0.308639953146409, y: 0.175, z: 1.22256279986678
        L1 (finger_1 -> finger_1_1): sqrt(0.179528059776071^2 + 0.0399996964134806^2) = 0.1839301496768
        L2 (finger_1_1 -> finger_1_1_1): 0.24480446144251752
        L3 (finger_1_1_1 -> finger_1_eef): 0.0406674787816

        GENERAL FK EQUATIONS:
        x = l1*cos(t1) - l3*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + l2*cos(t1)*cos(t2);
        y = l1*sin(t1) - l3*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)) + l2*cos(t2)*sin(t1);
        z = - l3*(cos(t2)*sin(t3) + cos(t3)*sin(t2)) - l2*sin(t2);
        */

        L1 = 0.1839301496768;
        L2 = 0.24480446144251752;
        L3 = 0.0406674787816;

        q1 = j_s.position[0];
        q2 = -(j_s.position[1]);
        q3 = (j_s.position[2]);

        f1_calc << 
            L1*cos(q1) - L3*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + L2*cos(q1)*cos(q2),
            L1*sin(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1),
            - L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2);

        f1_rotated = get_rotated(f1_calc, &f1_rotation[0]);

        f1_calculated[0] = L1*cos(q1) - L3*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + L2*cos(q1)*cos(q2);
        f1_calculated[1] = L1*sin(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1);
        f1_calculated[2] = - L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2);

        f1_estimated[0] = f1_calculated[0];// * cos(finger_inclination[0]) - f1_calculated[2] * sin(finger_inclination[0]);
        f1_estimated[1] = f1_calculated[1];
        f1_estimated[2] = f1_calculated[2];//f1_calculated[0] * sin(finger_inclination[0]) - f1_calculated[2] * cos(finger_inclination[0]);


        // FK Estimated Position of Finger 1's EEF
        eef1_estimated_marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(1).b(0).a(1); 
        eef1_estimated_marker.header.frame_id = "base_link";
        eef1_estimated_marker.id = 1;
        eef1_estimated_marker.type = 2;
        eef1_estimated_marker.action = 0;
        eef1_estimated_marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.025).y(0.025).z(0.025); 
        eef1_estimated_marker.pose.position.x = f1_rotated(0) + base_2_f1[0];
        eef1_estimated_marker.pose.position.y = f1_rotated(1) + base_2_f1[1];
        eef1_estimated_marker.pose.position.z = f1_rotated(2) + base_2_f1[2];
        eef1_estimated_marker_pub_->publish(eef1_estimated_marker);

        // Real Position of Finger 1's EEF
        eef1_marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(1).g(0).b(0).a(1); 
        eef1_marker.header.frame_id = "base_link";
        eef1_marker.id = 0;
        eef1_marker.type = 2;
        eef1_marker.action = 0;
        eef1_marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.025).y(0.025).z(0.025); 
        eef1_marker.pose.position.x = t.transform.translation.x;
        eef1_marker.pose.position.y = t.transform.translation.y;
        eef1_marker.pose.position.z = t.transform.translation.z;
        eef1_marker_pub_->publish(eef1_marker);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandControllerNode>());
    rclcpp::shutdown();
    return 0;
}
