#ifndef CUSTOM_DWA_PLANNER_H
#define CUSTOM_DWA_PLANNER_H

#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace custom_dwa_planner
{

class CustomDWAPlanner : public nav2_core::Controller
{
public:
    CustomDWAPlanner();
    ~CustomDWAPlanner() override = default; //destructor

    void configure( //configure the controller plugin
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(   //velocity commands based on current pose, velocity, and goal checker
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;    //global path for the controller
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override; //dynamic speed limit

private:
    void publishTrajectory();

    // Node and infrastructure
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::string plugin_name_;

    // Path and trajectory data
    nav_msgs::msg::Path global_path_;   //global path received from planner
    std::vector<std::tuple<double, double, double>> best_trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_traj_pub_;

    // Velocity constraints
    double max_vel_x_, max_vel_y_, max_vel_theta_;
    double sim_time_;
    double linear_granularity_, angular_granularity_;

    // Cost function weights
    double goal_weight_, obstacle_weight_, speed_weight_, smoothness_weight_;
};

} // namespace custom_dwa_planner

#endif // CUSTOM_DWA_PLANNER_H