#pragma once
#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_

#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/transport/Node.hh>
// #include <gz/msgs/msgs.hh>
// #include <gz/sensors/Sensor.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gazebo {

class PickupPlugin : public gz::sim::System,
                     public gz::sim::ISystemUpdate {
    public:
        PickupPlugin();
        virtual ~PickupPlugin();
        void Load(gz::sim::EntityComponentManager &_ecm, sdf::ElementPtr);

		// Update method called at every simulation step
        void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

    private:
        bool attach();
        bool detach();
        gz::sim::ModelPtr findNearbyObject();

        /// Node for ROS communication.
        gazebo_ros::Node::SharedPtr ros_node_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dockStatusPub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dockControlSub;

        rclcpp::TimerBase::SharedPtr initial_attaching_timer;

        gz::sim::ModelPtr model;
        gz::sim::WorldPtr world;
        gz::physics::PhysicsEnginePtr physics;
        gz::physics::JointPtr joint;
        gz::physics::LinkPtr sensor_link;

        std::string pickup_object_allowable_prefix = "";

        double allowable_offset_height = 0.15;
        double allowable_offset_horizontal = 0.2;

        int jointCounter;
    };

}

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
