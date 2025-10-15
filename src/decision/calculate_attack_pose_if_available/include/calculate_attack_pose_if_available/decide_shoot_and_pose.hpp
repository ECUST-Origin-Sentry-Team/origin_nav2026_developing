/*
 * @LastEditors: Tonytpc ykq31415926@163.com
 * @Date: 2025-09-19 15:00:39
 * @LastEditTime: 2025-09-28 22:06:55
 * @FilePath: decide_shoot_and_pose.hpp
 */
#ifndef DECIDE_SHOT_AND_POSE_HPP_
#define DECIDE_SHOT_AND_POSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/srv/invincible_inquire.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rm_interfaces/msg/is_able_to_chase_send.hpp"
#include "rm_interfaces/msg/is_able_to_chase_call.hpp"

#include <memory>

class DecideShootAndPose : public rclcpp::Node
{
public:
    DecideShootAndPose();

    void inquire_enemy_status(std::string &enemy_id, geometry_msgs::msg::Point enemy_pos);

private:
    rclcpp::Client<rm_interfaces::srv::InvincibleInquire>::SharedPtr inv_client_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<rm_interfaces::msg::IsAbleToChaseCall>::SharedPtr chase_pub_;
    rclcpp::Subscription<rm_interfaces::msg::IsAbleToChaseSend>::SharedPtr chase_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::TimerBase::SharedPtr map_to_gimbal_tf_timer;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster;

    void target_callback(const rm_interfaces::msg::Target::SharedPtr msg);
    void gimbal_cmd_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);
    void calculate_best_pose();
    void publish_enemy_marker(const geometry_msgs::msg::Point &position, const std::vector<geometry_msgs::msg::Point> &candidates, const geometry_msgs::msg::Point &best_point);

    std::vector<geometry_msgs::msg::Point> generateCandidatePoints(const geometry_msgs::msg::Point &position);
    std::vector<geometry_msgs::msg::Point> filterFeasiblePoints(const std::vector<geometry_msgs::msg::Point> &candidates, const nav_msgs::msg::OccupancyGrid &costmap);
    int lineCost(const nav_msgs::msg::OccupancyGrid &grid, int x0, int y0, int x1, int y1);
    geometry_msgs::msg::Point selectBestPoint(const std::vector<geometry_msgs::msg::Point> &feasible_points);

    const std::set<std::string> valid_values = {"1", "2", "3", "4", "sentry"};
    rm_interfaces::msg::GimbalCmd gimbal_cmd_msg_;
    float robot_position_x = -100;
    float robot_position_y = -100;
    float goal_x;
    float goal_y;
    float max_distance_from_goal = 0.0;
    geometry_msgs::msg::PointStamped enemy_pos_in_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap;
    geometry_msgs::msg::Point best_point;
    bool chase_ready = false;
    std::string base_link_frame_id;
    float attack_radius;
    int num_sectors;
    int cost_threshold;
    float max_chase_distance;
    bool chase_visualize;
};

#endif // DECIDE_SHOT_AND_POSE_HPP_
