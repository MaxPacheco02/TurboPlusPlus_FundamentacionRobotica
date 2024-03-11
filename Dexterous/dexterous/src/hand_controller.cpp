#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
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

        eef1_estimated_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/eef1_fk_marker", 10);
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/j_s", 10);

        updateTimer =
            this->create_wall_timer(100ms, std::bind(&HandControllerNode::update, this));

        temp_marker.header.frame_id = "base_link";
        temp_marker.id = 0;
        temp_marker.type = 2;
        temp_marker.action = 0;
        temp_marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.04).y(0.04).z(0.04); 
        
        j_s.name = std::vector<std::string>{"fin1", "fin1_1", "fin1_1_1", "fin2", "fin2_1", "fin2_1_1", "fin3", "fin3_1", "fin3_1_1"};

        base_to_f1 << 0.2921178340911865, 0.175, 1.1440668106079102;
        base_to_f2 << 0.338191099465507, 0.0, 0.774632196937085;
        base_to_f3 << 0.292117822710978, -0.175, 1.14406683612052;
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr eef1_estimated_marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

    visualization_msgs::msg::Marker temp_marker;
    sensor_msgs::msg::JointState j_s;

    rclcpp::TimerBase::SharedPtr updateTimer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double L1{0.179528059776071};    // Distance finger_x -> finger_x_1
    double L2{0.235471940223929};    // Distance finger_x_1 -> finger_x_1_1
    double L3{0.205};      // Distance finger_x_1_1 -> finger_x_eef

    double q1[3]{0, 0.1, 0.1};
    double q2[3]{0, 0.1, 0.1};
    double q3[3]{0, 0.1, 0.1};

    double kp = 1;
    double dt = 0.1;

    Eigen::Vector3f f1_fk, f2_fk, f3_fk;
    Eigen::Vector3f f_local;

    Eigen::Vector3f base_to_f1, base_to_f2, base_to_f3; // base_link to fingers translation
    double f1_rotations[3]{-3.14159265358979, -0.434081521732677, 0};  // finger 1 joint rotation
    double f2_rotations[3]{3.14159265358979, 0.194237175479863, 0};  // finger 2 joint rotation
    double f3_rotations[3]{-3.14159265358979, -0.434081521732677, 0};  // finger 3 joint rotation
    

    Eigen::Matrix3f get_rot_mat(double rot[3]){
        Eigen::Matrix3f rot_x, rot_y, rot_z;
        rot_x << 
            1, 0, 0,
            0, cos(rot[0]), sin(rot[0]),
            0, -sin(rot[0]), cos(rot[0]);
        rot_y << 
            cos(rot[1]), 0, -sin(rot[1]),
            0, 1, 0,
            sin(rot[1]), 0, cos(rot[1]);
        rot_z << 
            cos(rot[2]), -sin(rot[2]), 0,
            sin(rot[2]), cos(rot[2]), 0,
            0, 0, 1;
        return rot_x * rot_y * rot_z;
    }

    std::vector<Eigen::Vector3f> get_fk_finger(double qs[], Eigen::Matrix3f rot_m){
        Eigen::Vector3f fin;
        std::vector<Eigen::Vector3f> finger_list;
        double q1 = qs[0], q2 = qs[1], q3 = qs[2];

        fin << 
            L1*cos(q1) - L3*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + L2*cos(q1)*cos(q2),
            L1*sin(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1),
            L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + L2*sin(q2);
        finger_list.push_back(fin);

        RCLCPP_ERROR(
            this->get_logger(), "Finger Local Pose: %f, %f, %f",
            fin(0), fin(1), fin(2));

        finger_list.push_back(rot_m * fin);
        return finger_list;
    }
    
    Eigen::Vector3f update_q_ik(Eigen::Vector3f err, double q1, double q2, double q3){
        // Jacobian Matrix (Calculated from the partial derivative of forward kinematics)
        Eigen::Matrix3f J;
        J << 
            L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - L1*sin(q1) - L2*cos(q2)*sin(q1), 
            - L3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - L2*cos(q1)*sin(q2), 
            -L3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)),

            L1*cos(q1) - L3*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + L2*cos(q1)*cos(q2), 
            - L3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - L2*sin(q1)*sin(q2), 
            -L3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)),

            0, 
            L3*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + L2*cos(q2), 
            L3*(cos(q2)*cos(q3) - sin(q2)*sin(q3));

        Eigen::Vector3f theta_d, q_dot, new_q;
        theta_d << err(0) * kp, err(1) * kp, err(2) * kp;
        
        Eigen::Matrix3f j_inverse;
        bool invertible;
        float determinant;
        J.computeInverseAndDetWithCheck(j_inverse, determinant, invertible);
        if(invertible)
            q_dot = j_inverse * theta_d;
        else
            q_dot << 0, 0, 0;

        new_q << q1 + q_dot(0) * dt, q2 + q_dot(1) * dt, q3 + q_dot(2) * dt;
        return new_q;
    }

    void update() {        
        std::vector<Eigen::Vector3f> f1_data, f2_data, f3_data;
        Eigen::Vector3f goal_f1, error_f1, new_q1;

        f1_data = get_fk_finger(q1, get_rot_mat(f1_rotations));
        f_local = f1_data[0];
        f1_fk = f1_data[1] + base_to_f1;

        // // FK Estimated Position of Finger 1's EEF
        // temp_marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(1).b(0).a(1); 
        // temp_marker.pose.position.x = f1_fk(0);
        // temp_marker.pose.position.y = f1_fk(1);
        // temp_marker.pose.position.z = f1_fk(2);
        // eef1_estimated_marker_pub_->publish(temp_marker);

        goal_f1 << 0.8, 0.2, 1;

        // Error = Inverse Rotation Matrix * (the goal from the base of the finger) - current pose
        error_f1 = get_rot_mat(f1_rotations).inverse() * (goal_f1 - base_to_f1) - f_local;
        

        if(error_f1.norm() > 0.001){
            new_q1 = update_q_ik(error_f1, q1_1, q1_2, q1_3);
            q1_1 = new_q1(0);
            q1_2 = new_q1(1);
            q1_3 = new_q1(2);
        }

        q1_1 = std::clamp(q1_1, -1.0, 1.0);
        q1_2 = std::clamp(q1_2, -0.7, 2.3);
        q1_3 = std::clamp(q1_3, -0.5, 2.3);

        RCLCPP_ERROR(
        this->get_logger(), "EEF1 Pose: %f, %f, %f",
        f1_fk(0), f1_fk(1), f1_fk(2)
        );
        j_s.header.stamp = HandControllerNode::now();
        j_s.position = std::vector<double>{q1_1, q1_2, q1_3, q2_1, q2_2, q2_3, q3_1, q3_2, q3_3};
        joint_states_pub_->publish(j_s);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandControllerNode>());
    rclcpp::shutdown();
    return 0;
}


/*

ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: {fin1, fin1_1, fin1_1_1, fin2, fin2_1, fin2_1_1, fin3, fin3_1, fin3_1_1}, position: {-0.35140000000000005, 0.6594, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07}}"

header:
  stamp:
    sec: 1710059168
    nanosec: 538909729
  frame_id: ''
name:
- fin1
- fin1_1
- fin1_1_1
- fin2
- fin2_1
- fin2_1_1
- fin3
- fin3_1
- fin3_1_1
position:
- -0.35140000000000005
- 0.6594
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
velocity: []
effort: []

ros2 topic pub -r 100 /joint_states sensor_msgs/msg/JointState "{"header": {"stamp": {"sec": 8, "nanosec": 10001}, "frame_id": ""}, "name": ["fin1"], "position": [0], "velocity": [0], "effort": [0]}"
*/