#ifndef ARENA_HUMAN_PLUGIN__HUMANSYSTEMPLUGIN_H_
#define ARENA_HUMAN_PLUGIN__HUMANSYSTEMPLUGIN_H_

// C++ 
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <unordered_map>

// Gazebo 
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Actor.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>

// ROS 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "arena_people_msgs/msg/pedestrian.hpp"
#include "arena_people_msgs/msg/pedestrians.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace arena_human_plugin
{

class HumanSystemPlugin : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Constructor
  HumanSystemPlugin();

  /// \brief Destructor
  ~HumanSystemPlugin() override;

  /// \brief 
  void Configure(const gz::sim::Entity& _entity, 
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 gz::sim::EntityComponentManager& _ecm, 
                 gz::sim::EventManager& _eventMgr) final;

  /// \brief PreUpdate 
  virtual void PreUpdate(const gz::sim::UpdateInfo& _info, 
                         gz::sim::EntityComponentManager& _ecm) override;

private:
  /// \brief Callback for arena_peds topic
  void pedestriansCallback(const arena_people_msgs::msg::Pedestrians::SharedPtr msg);

  /// \brief Initialize agents 
  void initializeAgents(gz::sim::EntityComponentManager& _ecm);

  /// \brief Update Gazebo pedestrians 
  void updateGazeboPedestrians(gz::sim::EntityComponentManager& _ecm, 
                               const gz::sim::UpdateInfo& _info, 
                               const arena_people_msgs::msg::Pedestrians& _pedestrians);

  /// \brief Get world pose 
  gz::math::Pose3d worldPose(gz::sim::Entity entity,
                             const gz::sim::EntityComponentManager& _ecm) const;

  /// \brief Normalize angle (EXACT same as HuNavSystemPlugin)
  inline double normalizeAngle(double a)
  {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }

private:
  /// ROS node 
  rclcpp::Node::SharedPtr rosnode_;

  /// TF broadcaster 
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// Subscriber for arena_peds topic
  rclcpp::Subscription<arena_people_msgs::msg::Pedestrians>::SharedPtr pedestrians_sub_;

  /// Current pedestrian states
  arena_people_msgs::msg::Pedestrians::SharedPtr current_pedestrians_;

  /// Map of pedestrian agents 
  std::unordered_map<gz::sim::Entity, arena_people_msgs::msg::Pedestrian> pedestrians_;

  /// Gazebo entities 
  gz::sim::Entity worldEntity_;
  sdf::ElementPtr sdf_;

  /// Configuration 
  std::string namespace_;
  std::string global_frame_;
  std::string pedestrians_topic_;
  double update_rate_secs_;
  
  /// Flags 
  bool agents_initialized_;
  bool first_update_;
  int counter_;

  /// Timing 
  std::chrono::steady_clock::time_point last_update_time_;
};

}  // namespace arena_human_plugin

#endif  // ARENA_HUMAN_PLUGIN__HUMANSYSTEMPLUGIN_H_