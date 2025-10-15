/*
 * @LastEditors: Tonytpc ykq31415926@163.com
 * @Date: 2025-09-19 15:01:30
 * @LastEditTime: 2025-09-29 07:09:34
 * @FilePath: decide_shoot_and_pose.cpp
 */
#include "calculate_attack_pose_if_available/decide_shoot_and_pose.hpp"

DecideShootAndPose::DecideShootAndPose()
    : Node("decide_shot_and_pose")
{
    this->declare_parameter("base_link_frame_id", "base_link");
    this->declare_parameter("attack_radius", 3.0);
    this->declare_parameter("num_sectors", 15);
    this->declare_parameter("cost_threshold", 20);
    this->declare_parameter("max_chase_distance", 8.0);
    this->declare_parameter("chase_visualize", true);

    this->get_parameter("base_link_frame_id", base_link_frame_id);
    this->get_parameter("attack_radius", attack_radius);
    this->get_parameter("num_sectors", num_sectors);
    this->get_parameter("cost_threshold", cost_threshold);
    this->get_parameter("max_chase_distance", max_chase_distance);
    this->get_parameter("chase_visualize", chase_visualize);

    tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer);
    tf2_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    inv_client_ = this->create_client<rm_interfaces::srv::InvincibleInquire>("/enemy_invincible_status");
    target_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
        "/front/armor_solver/target", rclcpp::SensorDataQoS(),
        std::bind(&DecideShootAndPose::target_callback, this, std::placeholders::_1));
    gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/process_gimbal", rclcpp::SensorDataQoS(),
        std::bind(&DecideShootAndPose::gimbal_cmd_callback, this, std::placeholders::_1));
    gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
        "serial/process_gimbal_aft_inv", rclcpp::SensorDataQoS());
    map_to_gimbal_tf_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&DecideShootAndPose::calculate_best_pose, this));
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        // "/global_costmap/costmap",
        "/map",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            global_costmap = msg;
        });
    chase_pub_ = this->create_publisher<rm_interfaces::msg::IsAbleToChaseCall>(
        "/tree/chase_goal", rclcpp::QoS(rclcpp::KeepLast(10)));
    chase_sub_ = this->create_subscription<rm_interfaces::msg::IsAbleToChaseSend>(
        "/tree/able_chase",
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const rm_interfaces::msg::IsAbleToChaseSend::SharedPtr msg)
        {
            goal_x = msg->x;
            goal_y = msg->y;
            max_distance_from_goal = msg->max_distance_from_goal;
        });
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("enemy_markers", 10);
}

void DecideShootAndPose::inquire_enemy_status(std::string &enemy_id, geometry_msgs::msg::Point enemy_pos)
{
    if (!inv_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_WARN(this->get_logger(), "Service /enemy_invincible_status not available.");
        gimbal_cmd_pub_->publish(gimbal_cmd_msg_);
        return;
    }

    auto request = std::make_shared<rm_interfaces::srv::InvincibleInquire::Request>();
    request->enemy_id = enemy_id;

    using ServiceResponseFuture =
        rclcpp::Client<rm_interfaces::srv::InvincibleInquire>::SharedFuture;

    auto response_callback = [this, enemy_id, enemy_pos](ServiceResponseFuture future)
    {
        auto response = future.get();
        if (response->attack_available)
        {
            RCLCPP_INFO(this->get_logger(), "Enemy %s is attackable, shooting!", enemy_id.c_str());
            gimbal_cmd_pub_->publish(gimbal_cmd_msg_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Enemy %s is NOT attackable.", enemy_id.c_str());
            gimbal_cmd_msg_.pitch = 0.0;
            gimbal_cmd_msg_.yaw = 0.0;
            gimbal_cmd_msg_.fire_advice = false;
            gimbal_cmd_pub_->publish(gimbal_cmd_msg_);
        }
    };

    inv_client_->async_send_request(request, response_callback);
}

void DecideShootAndPose::target_callback(const rm_interfaces::msg::Target::SharedPtr msg)
{
    if (msg->id == "")
    {
        gimbal_cmd_msg_.pitch = 0.0;
        gimbal_cmd_msg_.yaw = 0.0;
        gimbal_cmd_msg_.fire_advice = false;
        gimbal_cmd_pub_->publish(gimbal_cmd_msg_);
    }
    else if (valid_values.count(msg->id))
    {
        inquire_enemy_status(msg->id, msg->position);
    }
    else
    {
        gimbal_cmd_pub_->publish(gimbal_cmd_msg_);
    }
}

void DecideShootAndPose::gimbal_cmd_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
    gimbal_cmd_msg_ = *msg;
}

void DecideShootAndPose::calculate_best_pose()
{
    auto msg = rm_interfaces::msg::IsAbleToChaseCall();
    msg.chase_ready = false;
    msg.chase_x = 0.0;
    msg.chase_y = 0.0;
    try
    {
        geometry_msgs::msg::TransformStamped map_to_gimbal_link = tf2_buffer->lookupTransform("map", base_link_frame_id, rclcpp::Time(0));
        robot_position_x = map_to_gimbal_link.transform.translation.x;
        robot_position_y = map_to_gimbal_link.transform.translation.y;

        if (gimbal_cmd_msg_.distance < max_chase_distance && gimbal_cmd_msg_.fire_advice == true)
        {

            // get enemy position
            geometry_msgs::msg::PointStamped enemy_pos_in_baselink;
            enemy_pos_in_baselink.header.frame_id = base_link_frame_id;
            enemy_pos_in_baselink.header.stamp = this->now();
            enemy_pos_in_baselink.point.x = gimbal_cmd_msg_.distance * cos(gimbal_cmd_msg_.pitch_diff / 180.0 * 3.14) * cos(gimbal_cmd_msg_.yaw_diff / 180.0 * 3.14);
            enemy_pos_in_baselink.point.y = gimbal_cmd_msg_.distance * cos(gimbal_cmd_msg_.pitch_diff / 180.0 * 3.14) * sin(gimbal_cmd_msg_.yaw_diff / 180.0 * 3.14);
            enemy_pos_in_baselink.point.z = gimbal_cmd_msg_.distance * sin(gimbal_cmd_msg_.pitch_diff / 180.0 * 3.14);
            geometry_msgs::msg::PointStamped enemy_pos_in_map;
            tf2::doTransform(enemy_pos_in_baselink, enemy_pos_in_map, map_to_gimbal_link);

            // start chase condition
            if (((goal_x - robot_position_x) * (goal_x - robot_position_x) + (goal_y - robot_position_y) * (goal_y - robot_position_y)) > max_distance_from_goal * max_distance_from_goal)
            {
                chase_pub_->publish(msg);
                return;
            }
            if (!global_costmap)
            {
                RCLCPP_ERROR(this->get_logger(), "Missing required input: costmap_port");
                chase_pub_->publish(msg);
                return;
            }
            std::vector<geometry_msgs::msg::Point> candidates = generateCandidatePoints(enemy_pos_in_map.point);
            std::vector<geometry_msgs::msg::Point> feasible_points = filterFeasiblePoints(candidates, *global_costmap);

            // has feasible points
            if (feasible_points.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No feasible attack points found");
                chase_pub_->publish(msg);
                return;
            }
            best_point = selectBestPoint(feasible_points);

            msg.chase_ready = true;
            msg.chase_x = best_point.x;
            msg.chase_y = best_point.y;

            if (chase_visualize)
            {
                publish_enemy_marker(enemy_pos_in_map.point, feasible_points, best_point);
            }
        }
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform enemy position: %s", ex.what());
    }
    chase_pub_->publish(msg);
}

std::vector<geometry_msgs::msg::Point> DecideShootAndPose::generateCandidatePoints(const geometry_msgs::msg::Point &position)
{
    std::vector<geometry_msgs::msg::Point> candidates;
    candidates.reserve(num_sectors);

    for (int i = 0; i < num_sectors; ++i)
    {
        const double angle = i * 2 * M_PI / num_sectors;
        geometry_msgs::msg::Point p;
        p.x = position.x + attack_radius * cos(angle);
        p.y = position.y + attack_radius * sin(angle);
        p.z = 0.0;
        candidates.push_back(p);
    }
    return candidates;
}

std::vector<geometry_msgs::msg::Point> DecideShootAndPose::filterFeasiblePoints(
    const std::vector<geometry_msgs::msg::Point> &candidates, const nav_msgs::msg::OccupancyGrid &costmap)
{
    std::vector<geometry_msgs::msg::Point> feasible_points;
    const auto &info = costmap.info;
    int robot_position_x_in_map = static_cast<int>((robot_position_x - info.origin.position.x) / info.resolution);
    int robot_position_y_in_map = static_cast<int>((robot_position_y - info.origin.position.y) / info.resolution);
    for (const auto &p : candidates)
    {
        const int cell_x = static_cast<int>((p.x - info.origin.position.x) / info.resolution);
        const int cell_y = static_cast<int>((p.y - info.origin.position.y) / info.resolution);

        if (
            cell_x < 0 || cell_x >= static_cast<int>(info.width) || cell_y < 0 ||
            cell_y >= static_cast<int>(info.height))
        {
            continue;
        }

        const int index = cell_y * info.width + cell_x;
        const int8_t cost = costmap.data[index];
        if (cost >= 0 && cost <= cost_threshold && lineCost(costmap, cell_x, cell_y, robot_position_x_in_map, robot_position_y_in_map) < 100)
        {
            feasible_points.push_back(p);
        }
    }
    return feasible_points;
}

// Bresenham 直线算法 用于计算线上是否存在代价为100的障碍物
int DecideShootAndPose::lineCost(
    const nav_msgs::msg::OccupancyGrid &grid,
    int x0, int y0, int x1, int y1)
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int max_cost = 0;

    while (true)
    {
        // 检查是否越界
        if (x0 < 0 || y0 < 0 ||
            x0 >= static_cast<int>(grid.info.width) ||
            y0 >= static_cast<int>(grid.info.height))
        {
            return 100; // 越界当作障碍
        }

        int idx = y0 * grid.info.width + x0;
        int occ = grid.data[idx];
        if (occ > max_cost)
        {
            max_cost = occ;
        }

        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }

    return max_cost;
}

geometry_msgs::msg::Point DecideShootAndPose::selectBestPoint(
    const std::vector<geometry_msgs::msg::Point> &feasible_points)
{
    auto compare = [&](const auto &a, const auto &b)
    {
        const double dx1 = a.x - robot_position_x;
        const double dy1 = a.y - robot_position_y;
        const double dx2 = b.x - robot_position_x;
        const double dy2 = b.y - robot_position_y;
        return (dx1 * dx1 + dy1 * dy1) < (dx2 * dx2 + dy2 * dy2);
    };
    return *std::min_element(feasible_points.begin(), feasible_points.end(), compare);
}

void DecideShootAndPose::publish_enemy_marker(
    const geometry_msgs::msg::Point &position,
    const std::vector<geometry_msgs::msg::Point> &candidates,
    const geometry_msgs::msg::Point &best_point)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker enemy_marker;
    enemy_marker.header.frame_id = "map";
    enemy_marker.header.stamp = this->now();
    enemy_marker.ns = "enemy";
    enemy_marker.id = 0;
    enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
    enemy_marker.action = visualization_msgs::msg::Marker::ADD;

    enemy_marker.pose.position = position;
    enemy_marker.pose.orientation.w = 1.0;

    enemy_marker.scale.x = 0.2;
    enemy_marker.scale.y = 0.2;
    enemy_marker.scale.z = 0.2;

    enemy_marker.color.r = 1.0;
    enemy_marker.color.g = 0.0;
    enemy_marker.color.b = 0.0;
    enemy_marker.color.a = 1.0;

    enemy_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers.push_back(enemy_marker);

    int id_counter = 1;
    for (const auto &p : candidates)
    {
        visualization_msgs::msg::Marker candidate_marker;
        candidate_marker.header.frame_id = "map";
        candidate_marker.header.stamp = this->now();
        candidate_marker.ns = "candidates";
        candidate_marker.id = id_counter++;
        candidate_marker.type = visualization_msgs::msg::Marker::SPHERE;
        candidate_marker.action = visualization_msgs::msg::Marker::ADD;

        candidate_marker.pose.position = p;
        candidate_marker.pose.orientation.w = 1.0;

        candidate_marker.scale.x = 0.15;
        candidate_marker.scale.y = 0.15;
        candidate_marker.scale.z = 0.15;

        candidate_marker.color.r = 0.0;
        candidate_marker.color.g = 0.0;
        candidate_marker.color.b = 1.0;
        candidate_marker.color.a = 1.0;

        candidate_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        marker_array.markers.push_back(candidate_marker);
    }

    visualization_msgs::msg::Marker best_marker;
    best_marker.header.frame_id = "map";
    best_marker.header.stamp = this->now();
    best_marker.ns = "best_point";
    best_marker.id = id_counter++;
    best_marker.type = visualization_msgs::msg::Marker::SPHERE;
    best_marker.action = visualization_msgs::msg::Marker::ADD;

    best_marker.pose.position = best_point;
    best_marker.pose.orientation.w = 1.0;

    best_marker.scale.x = 0.18;
    best_marker.scale.y = 0.18;
    best_marker.scale.z = 0.18;

    best_marker.color.r = 0.0;
    best_marker.color.g = 1.0;
    best_marker.color.b = 0.0;
    best_marker.color.a = 1.0;

    best_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers.push_back(best_marker);

    marker_array_pub_->publish(marker_array);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecideShootAndPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
