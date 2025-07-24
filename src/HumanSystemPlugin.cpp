#include "arena_human_plugin/HumanSystemPlugin.h"

using namespace arena_human_plugin;

//////////////////////////////////////////////////
HumanSystemPlugin::HumanSystemPlugin()
  : update_rate_secs_(0.1),
    namespace_(""),
    global_frame_("map"),
    pedestrians_topic_("arena_peds"),
    agents_initialized_(false),
    first_update_(true),
    counter_(0)
{
}

//////////////////////////////////////////////////
HumanSystemPlugin::~HumanSystemPlugin()
{
}

//////////////////////////////////////////////////
void HumanSystemPlugin::Configure(const gz::sim::Entity& _entity,
                                  const std::shared_ptr<const sdf::Element>& _sdf,
                                  gz::sim::EntityComponentManager& _ecm,
                                  gz::sim::EventManager& _eventMgr)
{
  (void)_eventMgr;
  counter_ = 0;
  worldEntity_ = _entity;

  // Initialize ROS 2 (SAME as HuNavSystemPlugin)
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  std::string nodename = "human_system_plugin_" + std::to_string(_entity);
  rosnode_ = std::make_shared<rclcpp::Node>(nodename.c_str());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rosnode_);

  // Get plugin parameters (SAME as HuNavSystemPlugin)
  sdf_ = _sdf->Clone();

  if (sdf_->HasElement("namespace"))
    namespace_ = sdf_->Get<std::string>("namespace");
  else
    namespace_ = "/";

  if(namespace_.empty() || namespace_.back() != '/')
    namespace_.push_back('/');

  if (sdf_->HasElement("global_frame_to_publish"))
    global_frame_ = sdf_->Get<std::string>("global_frame_to_publish");
  else
    global_frame_ = "map";

  if (sdf_->HasElement("pedestrians_topic"))
    pedestrians_topic_ = sdf_->Get<std::string>("pedestrians_topic");

  if (sdf_->HasElement("update_rate"))
  {
    double update_rate = sdf_->Get<double>("update_rate");
    update_rate_secs_ = 1.0 / update_rate;
  }

  // Subscribe to arena_peds topic 
  std::string full_topic = namespace_ + pedestrians_topic_;
  
  pedestrians_sub_ = rosnode_->create_subscription<arena_people_msgs::msg::Pedestrians>(
    full_topic,
    rclcpp::QoS(10),
    std::bind(&HumanSystemPlugin::pedestriansCallback, this, std::placeholders::_1));

  // Initialize timing
  last_update_time_ = std::chrono::steady_clock::now();

  gzmsg << "HumanSystemPlugin configured - Namespace: " << namespace_ 
        << ", Topic: " << full_topic << ", Frame: " << global_frame_ << std::endl;
}

//////////////////////////////////////////////////
void HumanSystemPlugin::pedestriansCallback(const arena_people_msgs::msg::Pedestrians::SharedPtr msg)
{
  current_pedestrians_ = msg;
  
  RCLCPP_DEBUG(rosnode_->get_logger(), 
    "Received %zu pedestrians update", msg->pedestrians.size());
}

//////////////////////////////////////////////////
void HumanSystemPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                  gz::sim::EntityComponentManager& _ecm)
{
  // Spin ROS node 
  rclcpp::spin_some(rosnode_);

  // Initialize agents on first update 
  if (!agents_initialized_)
  {
    initializeAgents(_ecm);
    agents_initialized_ = true;
  }

  // Update pedestrians if we have data
  if (current_pedestrians_ && !current_pedestrians_->pedestrians.empty())
  {
    updateGazeboPedestrians(_ecm, _info, *current_pedestrians_);
  }

  first_update_ = false;
}

//////////////////////////////////////////////////
void HumanSystemPlugin::initializeAgents(gz::sim::EntityComponentManager& _ecm)
{
  pedestrians_.clear();

  // Find all actor entities 
  _ecm.Each<gz::sim::components::Actor, gz::sim::components::Name>(
    [&](const gz::sim::Entity& agentEntity,
        const gz::sim::components::Actor* actorComp,
        const gz::sim::components::Name* name) -> bool
    {
      std::string actor_name = name->Data();
      
      // Create a pedestrian entry for this actor (ONLY for entity mapping)
      arena_people_msgs::msg::Pedestrian pedestrian;
      pedestrian.name = actor_name;
      pedestrian.id = static_cast<int>(agentEntity); // Use entity ID as pedestrian ID
      
      // Store ONLY the entity mapping - NO pose initialization
      // The actual pose data comes from arena_peds publisher
      pedestrians_[agentEntity] = pedestrian;
      
      gzmsg << "Initialized actor: " << actor_name << " (Entity: " << agentEntity << ")" << std::endl;
      return true;
    });

  gzmsg << "Initialized " << pedestrians_.size() << " pedestrian actors" << std::endl;
}

//////////////////////////////////////////////////
void HumanSystemPlugin::updateGazeboPedestrians(gz::sim::EntityComponentManager& _ecm,
                                                const gz::sim::UpdateInfo& _info,
                                                const arena_people_msgs::msg::Pedestrians& _pedestrians)
{
  for (const auto& pedestrian : _pedestrians.pedestrians)
  {
    // Find the corresponding actor entity by name
    gz::sim::Entity agentEntity = gz::sim::kNullEntity;
    
    for (const auto& [entity, stored_ped] : pedestrians_)
    {
      if (stored_ped.name == pedestrian.name)
      {
        agentEntity = entity;
        break;
      }
    }
    
    if (agentEntity == gz::sim::kNullEntity)
    {
      RCLCPP_WARN_THROTTLE(rosnode_->get_logger(), *rosnode_->get_clock(), 5000,
        "Actor not found for pedestrian: %s", pedestrian.name.c_str());
      continue;
    }


    // CRITICAL: Zero the local pose like HuNavSystemPlugin does
    // This prevents the SDF initial pose from acting as an offset
    auto lposeComp = _ecm.Component<gz::sim::components::Pose>(agentEntity);
    if (nullptr == lposeComp)
    {
      _ecm.CreateComponent(agentEntity, gz::sim::components::Pose(gz::math::Pose3d::Zero));
    }
    else
    {
      auto p = lposeComp->Data();
      auto newPose = p;
      newPose.Pos().X(0);  // Zero X position
      newPose.Pos().Y(0);  // Zero Y position
      *lposeComp = gz::sim::components::Pose(newPose);
    }
    _ecm.SetChanged(agentEntity, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);
    // Create actor pose from pedestrian data 
    gz::math::Pose3d actorPose;
    actorPose.Pos().X(pedestrian.position.position.x);
    actorPose.Pos().Y(pedestrian.position.position.y);
    actorPose.Pos().Z(0.65); // Height correction
    
    // Convert quaternion to yaw
    tf2::Quaternion quat(
      pedestrian.position.orientation.x,
      pedestrian.position.orientation.y,
      pedestrian.position.orientation.z,
      pedestrian.position.orientation.w
    );
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    actorPose.Rot() = gz::math::Quaterniond(0, 0.0, yaw);

    // UPDATE TRAJECTORY POSE 
    auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(agentEntity);
    if (trajPoseComp) {
      *trajPoseComp = gz::sim::components::TrajectoryPose(actorPose);
      _ecm.SetChanged(agentEntity, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);
    } else {
      _ecm.CreateComponent(agentEntity, gz::sim::components::TrajectoryPose(actorPose));
    }

    // UPDATE WORLD POSE 
    auto wpose = _ecm.Component<gz::sim::components::WorldPose>(agentEntity);
    if (wpose) {
      wpose->Data() = actorPose;
      _ecm.SetChanged(agentEntity, gz::sim::components::WorldPose::typeId, gz::sim::ComponentState::OneTimeChange);
    } else {
      _ecm.CreateComponent(agentEntity, gz::sim::components::WorldPose(actorPose));
    }

    // TF Broadcasting 
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = rosnode_->get_clock()->now();
    tf_msg.header.frame_id = global_frame_;
    tf_msg.child_frame_id = "agent_" + pedestrian.name;
    
    tf_msg.transform.translation.x = actorPose.Pos().X();
    tf_msg.transform.translation.y = actorPose.Pos().Y();
    tf_msg.transform.translation.z = actorPose.Pos().Z();
    
    tf_msg.transform.rotation.x = actorPose.Rot().X();
    tf_msg.transform.rotation.y = actorPose.Rot().Y();
    tf_msg.transform.rotation.z = actorPose.Rot().Z();
    tf_msg.transform.rotation.w = actorPose.Rot().W();
    
    // Publish the transform
    tf_broadcaster_->sendTransform(tf_msg);

    // Update stored pedestrian data
    pedestrians_[agentEntity] = pedestrian;

    RCLCPP_DEBUG(rosnode_->get_logger(),
      "Updated actor %s: pos[%.2f, %.2f, %.2f]",
      pedestrian.name.c_str(), 
      actorPose.Pos().X(), actorPose.Pos().Y(), actorPose.Pos().Z());

    // RCLCPP_ERROR(rosnode_->get_logger(), 
    //   "Actor %s: Input[%.3f, %.3f] -> Gazebo[%.3f, %.3f] -> Offset[%.3f, %.3f]", 
    //   pedestrian.name.c_str(),
    //   pedestrian.position.position.x, pedestrian.position.position.y,
    //   actorPose.Pos().X(), actorPose.Pos().Y(),
    //   actorPose.Pos().X() - pedestrian.position.position.x,
    //   actorPose.Pos().Y() - pedestrian.position.position.y);
      }
}

//////////////////////////////////////////////////
gz::math::Pose3d HumanSystemPlugin::worldPose(gz::sim::Entity entity,
                                              const gz::sim::EntityComponentManager& _ecm) const
{
  
  auto poseComp = _ecm.Component<gz::sim::components::WorldPose>(entity);
  if (poseComp)
  {
    return poseComp->Data();
  }
  
  // Fallback to local pose
  auto localPoseComp = _ecm.Component<gz::sim::components::Pose>(entity);
  if (localPoseComp)
  {
    return localPoseComp->Data();
  }
  
  return gz::math::Pose3d::Zero;
}

// Register the plugin 
GZ_ADD_PLUGIN(HumanSystemPlugin,gz::sim::System,HumanSystemPlugin::ISystemConfigure,HumanSystemPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(HumanSystemPlugin, "HumanSystemPlugin")