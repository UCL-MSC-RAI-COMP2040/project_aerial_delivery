#include "dock_sim.hpp"

#include <gz/plugin/Register.hh>
// Register the plugin with Gazebo Fortress
IGNITION_ADD_PLUGIN(
    gzplugin::PickupPlugin,
    gz::sim::System,
	gzplugin::PickupPlugin::ISystemConfigure)

// Optional alias for the plugin
IGNITION_ADD_PLUGIN_ALIAS(gzplugin::PickupPlugin, "gzplugin::PickupPlugin")

// using namespace ignition;
namespace gzplugin
{
	
// Constructor
PickupPlugin::PickupPlugin()
//   : model(nullptr),
    // world(nullptr)
// 	physics(nullptr),
// 	joint(nullptr),
// 	jointCounter(0)
{
}


// Destructor
PickupPlugin::~PickupPlugin() {}

// void PickupPlugin::Load(gz::sim::EntityComponentManager &_ecm, sdf::ElementPtr _sdf) {
  // Implement Configure callback, provided by ISystemConfigure
  // and called once at startup.
void PickupPlugin::Configure(const gz::sim::Entity &_entity,
				const std::shared_ptr<const sdf::Element> &_sdf,
				gz::sim::EntityComponentManager &_ecm,
				gz::sim::EventManager &/*_eventMgr*/)
{

	// // Create a GazeboRos node instead of a common ROS node.
	// // Pass it SDF parameters so common options like namespace and remapping
	// // can be handled. 
	// Not needed anymore as ROS not integrated as closely
	// ros_node_ = gazebo_ros::Node::Get(_sdf);

	this->ecm = &_ecm;
	// // Obtain the model entity (model pointer is no longer passed directly)
    // gz::sim::Entity modelEntity = gz::sim::Model(_sdf->GetParent());
    this->model = gz::sim::Model(_entity); //std::make_shared<gz::physics::Model>(modelEntity);

    // // Retrieve the world entity
    this->world = gz::sim::World(_entity);

	// // Get Physics	
	// this->physics = this->world->Physics();

	// // // Get parameters specified in the sdf file.
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
	this->sensor_link = gz::sim::Link(model.LinkByName(_ecm, sensor_link_name));
	if(this->sensor_link.Entity() == gz::sim::kNullEntity) {
		ignmsg << "Sensor link [" << sensor_link_name << "] does not exist, defaulting to base link" << std::endl;
		this->sensor_link = gz::sim::Link(model.CanonicalLink(_ecm));
	}

	// // Create Publisher and Subscriber for ROS2 to interact with this
	this->dockStatusPub = node.Advertise<ignition::msgs::Boolean>("status");
	this->node.Subscribe("control", &PickupPlugin::OnControlMsg, this);

	ignmsg << "Publisher initialised on status" << std::endl;
	ignmsg << "Subscriber initialised on control" << std::endl;
	// RCLCPP_INFO(this->ros_node_->get_logger(), "Publisher initialised on [%s]", this->dockStatusPub->get_topic_name());
	// RCLCPP_INFO(this->ros_node_->get_logger(), "Subscriber initialised on [%s]", this->dockControlSub->get_topic_name());

	// bool initially_attach = true;
	// if(initially_attach) {
	// 	RCLCPP_INFO(this->ros_node_->get_logger(), "Attempting to initially attach");
	// 	this->initial_attaching_timer = this->ros_node_->create_wall_timer(std::chrono::duration<double>(0.1), [this](){this->attach();});
	// }
}


void PickupPlugin::OnControlMsg(const ignition::msgs::Boolean &_msg)
{
	if (_msg.data()) {
		auto result = this->attach();
		ignition::msgs::Boolean status_msg;
		status_msg.set_data(result);
		this->dockStatusPub.Publish(status_msg);
	} else {
		auto result = this->detach();
		ignition::msgs::Boolean status_msg;
		status_msg.set_data(!result); // status of false means detached
		this->dockStatusPub.Publish(status_msg);
	}
}

// void PickupPlugin::Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
// {
//     // Your logic to check distance or conditions for attaching/detaching joints
//     // Example: if (distance < threshold) { attachJoint(); }
// }

// Check for objects within a certain vertical and horizontal offset of the drone
gz::sim::Entity PickupPlugin::findNearbyObject(){
	
	std::optional<gz::math::Pose3d> dronePose;
	if (this->sensor_link.Entity() != gz::sim::kNullEntity) {
		dronePose = this->sensor_link.WorldPose(*this->ecm);
	} else {
		return gz::sim::kNullEntity;
	}

	gz::math::Vector3d dronePos;
	if(dronePose) {
		dronePos = (*dronePose).Pos();
	} else {
		return gz::sim::kNullEntity;	
	}
	

	for(gz::sim::Entity ents: this->world.Models(*this->ecm)) {
		auto obj = gz::sim::Model(ents);

		if (obj.Entity() == this->model.Entity()) continue;

		if (obj.Name(*this->ecm).find(this->pickup_object_allowable_prefix) != 0) continue;

		auto canonLink = gz::sim::Link(obj.CanonicalLink(*this->ecm));
		auto pose = canonLink.WorldPose(*this->ecm);
		if(!pose){ continue; }

		gz::math::Vector3d objPos = (*pose).Pos();
		double verticalOffset = dronePos.Z() - objPos.Z();
		double horizontalOffset = hypot(objPos.X() - dronePos.X(), objPos.Y() - dronePos.Y());

		if (verticalOffset > 0
			&& verticalOffset <= this->allowable_offset_height 
			&& horizontalOffset <= this->allowable_offset_horizontal)
		{
			return obj.Entity();
		}
	}
	return gz::sim::kNullEntity;
}


bool PickupPlugin::attach() {
// 	// Find the payload
	gz::sim::Entity ePayload = this->findNearbyObject();
	if( ePayload == gz::sim::kNullEntity ) {
		// Nothing found matching	
		return false;
	} 

	gz::sim::Model payload = gz::sim::Model(ePayload);
	ignmsg << "Found Pickup Target " << payload.Name(*this->ecm) << std::endl;
	// 		RCLCPP_INFO(this->ros_node_->get_logger(), "Found Pickup Target [%s]", payload->GetName().c_str());
	

	// 	// RCLCPP_INFO(this->ros_node_->get_logger(), "getting drone base link");
	ignmsg << "getting drone base link" << std::endl;
	auto child_link = gz::sim::Link(this->model.CanonicalLink(*this->ecm));

	if (child_link.Entity() == gz::sim::kNullEntity){
		ignmsg << "child link from drone is none, here is a list of links" << std::endl;
		auto links1 = this->model.Links(*this->ecm);
		for(auto a: links1){
			auto name = gz::sim::Link(a).Name(*this->ecm);
			ignmsg << *name << std::endl;
		}
		return false;
	}

	// RCLCPP_INFO(this->ros_node_->get_logger(), "getting payload base link");
	ignmsg << "getting payload base link" << std::endl;
	auto parent_link = gz::sim::Link(payload.CanonicalLink(*this->ecm));
	if (parent_link.Entity() == gz::sim::kNullEntity){
		ignmsg << "parent link from payload is none, here is a list of links" << std::endl;
		auto links1 = payload.Links(*this->ecm);
		for(auto a: links1){
			auto name = gz::sim::Link(a).Name(*this->ecm);
			ignmsg << *name << std::endl;
		}
		return false;
	}

	if(joint.Entity() != gz::sim::kNullEntity ) {
		ignmsg << "Already Attached" << std::endl;
		// if (this->initial_attaching_timer) {
		// 	RCLCPP_INFO(this->ros_node_->get_logger(), "Initial attach succesful, timer cancelling");
		// 	this->initial_attaching_timer->cancel();
		// }
		return true;
	}

	// Test if links are within docking tolerance
	// RCLCPP_INFO(this->ros_node_->get_logger(), "Testing tolerance");
	ignmsg << "Testing Tolerance" << std::endl;
	auto poseOffset = *(parent_link.WorldPose(*this->ecm)) - *(child_link.WorldPose(*this->ecm));
	bool inToleranceHeight = poseOffset.Pos().Z() < this->allowable_offset_height;
	poseOffset.Pos().Z(0.0);
	bool inToleranceHorizontal = poseOffset.Pos().SquaredLength() < this->allowable_offset_horizontal;
	bool inTolerance = inToleranceHeight && inToleranceHorizontal;

	if(!inTolerance) {
		ignmsg << "Drone not within object tolerance " << poseOffset.Pos().SquaredLength() << std::endl;
		return false;
	}

// 	RCLCPP_INFO(this->ros_node_->get_logger(), "Creating new joint.");
	ignmsg << "Creating New Joint" << std::endl;
	std::stringstream jointName;
	jointName << "sim_dock_joint_" << jointCounter;

	auto joint_entity = this->ecm->CreateEntity();
	this->ecm->CreateComponent( 
		joint_entity,
		gz::sim::components::DetachableJoint({parent_link.Entity(), child_link.Entity(), "fixed"}));
	this->joint = gz::sim::Joint(joint_entity);

// 	joint = this->model->CreateJoint(jointName.str(),"fixed",parent_link,child_link);
// 	if( !joint ) {
// 		RCLCPP_ERROR(this->ros_node_->get_logger(), "Could not create joint");
// 		return false;
// 	}
	jointCounter++;
// 	joint->Load(parent_link,child_link, ignition::math::Pose3d());
// 	joint->Init();

// 	RCLCPP_INFO(this->ros_node_->get_logger(), "Attaching joint");
// 	joint->Attach(parent_link,child_link);
	return true;
}

bool PickupPlugin::detach() {
	if(this->joint.Entity() == gz::sim::kNullEntity) {
		// Joint doesn't exist so detached...
		return true;
	}
// 	RCLCPP_INFO(this->ros_node_->get_logger(), "Detaching joint");
	ignmsg << "Detaching Joint" << std::endl;
// 	this->joint->Fini();
	this->joint.~Joint();
	this->joint = gz::sim::Joint(gz::sim::kNullEntity);
	return true;
}

} // namespace gazebo
