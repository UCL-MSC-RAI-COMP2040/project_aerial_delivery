#pragma once
#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_

#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/math.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/JointType.hh>
#include <sdf/Joint.hh>
// #include <gz/msgs/msgs.hh>
// #include <gz/sensors/Sensor.hh>

// #include <gazebo_ros/node.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/bool.hpp>

// using namespace ignition;
// using namespace gazebo;
namespace gzplugin {

class PickupPlugin : public gz::sim::System,
                     public gz::sim::ISystemConfigure, //{
                     public gz::sim::ISystemPostUpdate { 
    public:
        PickupPlugin();
        virtual ~PickupPlugin();

        // Implement Configure callback, provided by ISystemConfigure
        // and called once at startup.
        void Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &/*_eventMgr*/) override;
            
        // void Load(gz::sim::EntityComponentManager &_ecm, sdf::ElementPtr);

		// Update method called at every simulation step
        // void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;
         
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private:
        bool attach();
        bool detach();
        void OnControlMsg(const ignition::msgs::Boolean &_msg);
        gz::sim::Entity findNearbyObject();

        // Node for trasnport to send to ROS2 eventually
        gz::transport::Node node;
        gz::transport::Node::Publisher dockStatusPub;

        // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dockStatusPub;
        // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dockControlSub;

        // rclcpp::TimerBase::SharedPtr initial_attaching_timer;
        
        gz::sim::EntityComponentManager *ecm{nullptr};
        gz::sim::Model model;
        gz::sim::World world;
        // gz::physics::PhysicsEnginePtr physics;
        gz::sim::Joint joint;
        gz::sim::Link sensor_link;

        std::string robot_namespace = "";
        std::string pickup_object_allowable_prefix = "";

        double allowable_offset_height = 0.15;
        double allowable_offset_horizontal = 0.2;

        int jointCounter = 0;

        bool _configured = false;
    };

};

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
