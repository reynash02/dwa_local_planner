#include "custom_dwa_planner/custom_dwa_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <cmath>
#include <algorithm>

namespace custom_dwa_planner
{

CustomDWAPlanner::CustomDWAPlanner() 
  : node_(nullptr), plugin_name_(""), tf_buffer_(nullptr), costmap_ros_(nullptr) {}

void CustomDWAPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    if (!node_) {
        throw std::runtime_error("Failed to lock LifecycleNode in CustomDWAPlanner::configure");
    }
    plugin_name_ = name;
    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;

    // Declare parameters
    auto declare_param = [&](const std::string& param, double default_val) {
        node_->declare_parameter(name + "." + param, default_val);
        return node_->get_parameter(name + "." + param).as_double();
    };

    max_vel_x_ = declare_param("desired_linear_vel", 0.5);
    max_vel_y_ = declare_param("max_vel_y", 0.0);
    max_vel_theta_ = declare_param("max_angular_vel", 1.0);
    sim_time_ = declare_param("lookahead_time", 1.0);
    linear_granularity_ = declare_param("linear_granularity", 0.05);
    angular_granularity_ = declare_param("angular_granularity", 0.1);
    goal_weight_ = declare_param("goal_weight", 1.0);
    obstacle_weight_ = declare_param("obstacle_weight", 1.0);
    speed_weight_ = declare_param("speed_weight", 0.5);
    smoothness_weight_ = declare_param("smoothness_weight", 0.5);

    selected_traj_pub_ = node_->create_publisher<nav_msgs::msg::Path>("dwa_selected_trajectory", 10);
    RCLCPP_INFO(node_->get_logger(), "Custom DWA Planner configured: %s", plugin_name_.c_str());
}

void CustomDWAPlanner::activate() {}
void CustomDWAPlanner::deactivate() {}
void CustomDWAPlanner::cleanup() {}

void CustomDWAPlanner::setPlan(const nav_msgs::msg::Path & path)
{
    global_path_ = path;
    RCLCPP_INFO(node_->get_logger(), "Received new path with %zu poses", path.poses.size());
}

geometry_msgs::msg::TwistStamped CustomDWAPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;

    auto costmap = costmap_ros_->getCostmap();
    const double robot_x = pose.pose.position.x;
    const double robot_y = pose.pose.position.y;
    const double robot_yaw = tf2::getYaw(pose.pose.orientation);

    double best_cost = 50000.0;
    double best_vx = 0.0, best_vth = 0.0;

    for (double vx = 0; vx <= max_vel_x_; vx += linear_granularity_) {
        for (double vth = -max_vel_theta_; vth <= max_vel_theta_; vth += angular_granularity_) {
            double x = robot_x, y = robot_y, yaw = robot_yaw;   //initialize simulation state with current robot pose
            double obstacle_cost = 0.0;
            bool collision_free = true;
            std::vector<std::tuple<double, double, double>> trajectory;

            // Simulate forward in time
            for (double t = 0; t < sim_time_; t += 0.1) {
                x += vx * cos(yaw) * 0.1;
                y += vx * sin(yaw) * 0.1;
                yaw += vth * 0.1;
                trajectory.emplace_back(x, y, yaw);

                unsigned int mx, my;
                if (!costmap->worldToMap(x, y, mx, my)) {
                    collision_free = false;
                    obstacle_cost += obstacle_weight_ * 100.0;
                    break;
                }

                double cost = costmap->getCost(mx, my);
                if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {    
                    collision_free = false;      // position is outside the map bounds
                    obstacle_cost += obstacle_weight_ * 100.0;  //high penalty for collision
                    break;
                } else if (cost > 0) {
                    obstacle_cost += obstacle_weight_ * cost;   //trajectory is near obstacle 
                }
            }

            if (!collision_free) continue;

            // Compute goal distance
            double goal_dist = std::numeric_limits<double>::max();  //find minimum distance from trajectory endpoint to global path
            for (const auto & path_pose : global_path_.poses) {
                double dx = x - path_pose.pose.position.x;
                double dy = y - path_pose.pose.position.y;
                goal_dist = std::min(goal_dist, std::hypot(dx, dy));
            }

            // Calculate total cost
            double total_cost = goal_weight_ * goal_dist + obstacle_cost +      //total cost as weighted sum of components
                              speed_weight_ * (max_vel_x_ - vx) +
                              smoothness_weight_ * std::abs(vth) * (0.1 + 0.2 * vx / max_vel_x_);

            if (total_cost < best_cost) {   //update best trajectory if this one has lower cost
                best_cost = total_cost;
                best_vx = vx;
                best_vth = vth;
                best_trajectory_ = trajectory;
            }
        }
    }

    cmd_vel.twist.linear.x = best_vx;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = best_vth;

    publishTrajectory();
    RCLCPP_INFO(node_->get_logger(), "Computed velocity: vx=%.2f, vth=%.2f", best_vx, best_vth);
    return cmd_vel;
}

void CustomDWAPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    if (percentage) {
        max_vel_x_ *= speed_limit;
        max_vel_theta_ *= speed_limit;
    } else {
        max_vel_x_ = speed_limit;
        max_vel_theta_ = speed_limit;
    }
}

void CustomDWAPlanner::publishTrajectory()      //publish the best trajectory as a Path message for RViz visualization
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (const auto & [x, y, yaw] : best_trajectory_) {     //convert trajectory points to PoseStamped
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;     //z always 0 for 2D plane
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose);
    }

    selected_traj_pub_->publish(path);  //publish trajectory path
}

} // namespace custom_dwa_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_dwa_planner::CustomDWAPlanner, nav2_core::Controller) //register class as a nav2 controller plugin