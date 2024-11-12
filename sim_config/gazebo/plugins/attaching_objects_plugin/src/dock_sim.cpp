#include "dock_sim.hpp"

// Register the plugin with Gazebo Fortress
GZ_ADD_PLUGIN(
    gazebo::PickupPlugin,
    gz::sim::System,
    gz::sim::ISystemUpdate)

// Optional alias for the plugin
// GZ_ADD_PLUGIN_ALIAS(gazebo::PickupPlugin, "gazebo::PickupPlugin")

namespace gazebo {

// Constructor
PickupPlugin::PickupPlugin()
  : model(nullptr),
    world(nullptr),
	physics(nullptr),
	joint(nullptr),
	jointCounter(0)
{
}


// Destructor
PickupPlugin::~PickupPlugin() {}

void PickupPlugin::Load(gz::sim::EntityComponentManager &_ecm, sdf::ElementPtr _sdf) {

	// Create a GazeboRos node instead of a common ROS node.
	// Pass it SDF parameters so common options like namespace and remapping
	// can be handled.
	ros_node_ = gazebo_ros::Node::Get(_sdf);

	// Obtain the model entity (model pointer is no longer passed directly)
    gz::sim::Entity modelEntity = gz::sim::Model(_sdf->GetParent());
    this->model = std::make_shared<gz::physics::Model>(modelEntity);

    // Retrieve the world entity
    this->world = std::make_shared<gz::sim::World

	// Get Physics	
	this->physics = this->world->Physics();

	// // Get parameters specified in the sdf file.
	if (_sdf->HasElement("allowable_offset_height")) {
		this->allowable_offset_height = _sdf->Get<double>("allowable_offset_height");
	}
	if (_sdf->HasElement("allowable_offset_horizontal")) {
		this->allowable_offset_horizontal = _sdf->Get<double>("allowable_offset_horizontal");
	}
	if (_sdf->HasElement("pickup_prefix")) {
		this->pickup_object_allowable_prefix = _sdf->Get<std::string>("pickup_prefix");
	}

	std::string sensor_link_name = "base_link";
	if (_sdf->HasElement("sensor_link")) {
		sensor_link_name = _sdf->Get<std::string>("sensor_link");
	}
	this->sensor_link = model->GetLink(sensor_link_name);
	if(!this->sensor_link) {
		RCLCPP_ERROR(this->ros_node_->get_logger(), "Sensor link [%s] does not exist, defaulting to base link", sensor_link_name.c_str());
	}

	// Create Publisher and Subscriber for ROS2 to interact with this
	this->dockStatusPub = this->ros_node_->create_publisher<std_msgs::msg::Bool>("status", 1);
	this->dockControlSub = this->ros_node_->create_subscription<std_msgs::msg::Bool>(
		"control",1,
		[this](const std_msgs::msg::Bool::SharedPtr dockControl) {
			if (dockControl->data) {
				auto result = this->attach();
				std_msgs::msg::Bool status_msg;
				status_msg.data = result;
				dockStatusPub->publish(status_msg);
			} else {
				auto result = this->detach();
				std_msgs::msg::Bool status_msg;
				status_msg.data = !result; // status of false means detached
				dockStatusPub->publish(status_msg);
			}
		}
	);

	RCLCPP_INFO(this->ros_node_->get_logger(), "Publisher initialised on [%s]", this->dockStatusPub->get_topic_name());
	RCLCPP_INFO(this->ros_node_->get_logger(), "Subscriber initialised on [%s]", this->dockControlSub->get_topic_name());

	// bool initially_attach = true;
	// if(initially_attach) {
	// 	RCLCPP_INFO(this->ros_node_->get_logger(), "Attempting to initially attach");
	// 	this->initial_attaching_timer = this->ros_node_->create_wall_timer(std::chrono::duration<double>(0.1), [this](){this->attach();});
	// }
}

void PickupPlugin::Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
    // Your logic to check distance or conditions for attaching/detaching joints
    // Example: if (distance < threshold) { attachJoint(); }
}

// Check for objects within a certain vertical and horizontal offset of the drone
physics::ModelPtr PickupPlugin::findNearbyObject(){
	ignition::math::Vector3d dronePos;
	if (this->sensor_link) {
		dronePos = this->sensor_link->WorldPose().Pos();
	} else {
		dronePos = this->model->WorldPose().Pos();
	}

	for(auto &obj: this->world->Models()) {
		if (obj == this->model) continue;

		if (obj->GetName().find(this->pickup_object_allowable_prefix) != 0) continue;

		ignition::math::Vector3d objPos = obj->WorldPose().Pos();

		double verticalOffset = dronePos.Z() - objPos.Z();
		double horizontalOffset = hypot(objPos.X() - dronePos.X(), objPos.Y() - dronePos.Y());

		if (verticalOffset > 0
			&& verticalOffset <= this->allowable_offset_height 
			&& horizontalOffset <= this->allowable_offset_horizontal)
		{
			return obj;
		}
	}
	return nullptr;
}


bool PickupPlugin::attach() {
	// Find the payload
	physics::ModelPtr payload = this->findNearbyObject();
	if( !payload ) {
		// Nothing found matching
		return false;
	} else {
		RCLCPP_INFO(this->ros_node_->get_logger(), "Found Pickup Target [%s]", payload->GetName().c_str());
	}

	// RCLCPP_INFO(this->ros_node_->get_logger(), "getting drone base link");
	auto child_link = this->model->GetLink("base_link");

	if (!child_link){
		RCLCPP_ERROR(this->ros_node_->get_logger(), "child link from drone is none, here is a list of links");
		auto links1 = this->model->GetLinks();
		for(auto a: links1){
			RCLCPP_INFO(this->ros_node_->get_logger(), a->GetName().c_str());
		}
		return false;
	}

	// RCLCPP_INFO(this->ros_node_->get_logger(), "getting payload base link");
	auto parent_link = payload->GetLink("base_link");

	if (!parent_link){
		RCLCPP_INFO(this->ros_node_->get_logger(), "parent link from payload is none, here is a list of links");
		auto links = payload->GetLinks();
		for(auto a: links){
				RCLCPP_INFO(this->ros_node_->get_logger(), a->GetName().c_str());
		}
		return false;
	}

	if( this->joint ) {
		RCLCPP_INFO(this->ros_node_->get_logger(), "Already attached.");
		if (this->initial_attaching_timer) {
			RCLCPP_INFO(this->ros_node_->get_logger(), "Initial attach succesful, timer cancelling");
			this->initial_attaching_timer->cancel();
		}
		return true;
	}


	// Test if links are within docking tolerance
	// RCLCPP_INFO(this->ros_node_->get_logger(), "Testing tolerance");
	// auto poseOffset = parent_link->WorldPose() - child_link->WorldPose();
	// bool inToleranceHeight = poseOffset.Pos().Z() < this->allowable_offset_height;
	// poseOffset.Pos().Z(0.0);
	// bool inToleranceHorizontal = poseOffset.Pos().SquaredLength() < this->allowable_offset_horizontal;
	// bool inTolerance = inToleranceHeight && inToleranceHorizontal;

	// if(!inTolerance) {
	// 	RCLCPP_WARN(this->ros_node_->get_logger(), "Drone not within object tolerance %f", poseOffset.Pos().SquaredLength());
	// 	return false;
	// }

	RCLCPP_INFO(this->ros_node_->get_logger(), "Creating new joint.");
	std::stringstream jointName;
	jointName << "sim_dock_joint_" << jointCounter;
	joint = this->model->CreateJoint(jointName.str(),"fixed",parent_link,child_link);
	if( !joint ) {
		RCLCPP_ERROR(this->ros_node_->get_logger(), "Could not create joint");
		return false;
	}
	jointCounter++;
	joint->Load(parent_link,child_link, ignition::math::Pose3d());
	joint->Init();

	RCLCPP_INFO(this->ros_node_->get_logger(), "Attaching joint");
	joint->Attach(parent_link,child_link);
	return true;
	}

bool PickupPlugin::detach() {
	if(!this->joint) {
		// Joint doesn't exist so detached...
		return true;
		}
	RCLCPP_INFO(this->ros_node_->get_logger(), "Detaching joint");
	this->joint->Fini();
	this->joint->~Joint();
	this->joint = nullptr;
	return true;
	}

} // namespace gazebo
