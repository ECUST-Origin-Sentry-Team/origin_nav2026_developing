#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rm_interfaces/msg/gimbal_region_cmd.hpp"
#include <nlohmann/json.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Eigen>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

class RegionBehaviorNode : public rclcpp::Node
{
public:
    RegionBehaviorNode() : Node("region_behavior_node")
    {
        this->declare_parameter("map_frame_id", "map");
        this->get_parameter("map_frame_id", map_frame_id_);
        this->declare_parameter("base_link_frame_id", "base_link");
        this->get_parameter("base_link_frame_id", base_link_frame_id_);

        std::string pkg_dir = ament_index_cpp::get_package_share_directory("region_behavior");
        std::string default_path = pkg_dir + "/config/1.json";
        this->declare_parameter("json_path", default_path);
        this->get_parameter("json_path", json_path_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        if (!load_regions(json_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "区域配置加载失败，节点退出。");
            rclcpp::shutdown();
            return;
        }

        region_int_pub_ = this->create_publisher<std_msgs::msg::Int32>("/region_int", 10);
        gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalRegionCmd>("/serial/gimbal_region_cmd", 10);

        arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "/direction_arrow", 1);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RegionBehaviorNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "节点启动完成。");
    }

private:
    struct LongEdge
    {
        Eigen::Vector2d start;
        Eigen::Vector2d end;
    };
    struct Region
    {
        std::string id;
        int type;
        std::vector<geometry_msgs::msg::Point> points;
        LongEdge long_edge;
    };

    std::vector<Region> regions_;
    std::string map_frame_id_;
    std::string base_link_frame_id_;
    std::string json_path_;
    Eigen::Vector2d dir;

    std::unique_ptr<tf2_ros::Buffer>
        tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub;
    rclcpp::Publisher<rm_interfaces::msg::GimbalRegionCmd>::SharedPtr gimbal_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr region_int_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rm_interfaces::msg::GimbalRegionCmd region_cmd_;
    geometry_msgs::msg::Point curr_pos_;
    bool in_region_ = false;
    float in_region_angle = 0.0;

    bool load_regions(const std::string &json_path)
    {

        std::ifstream file(json_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开JSON文件: %s", json_path.c_str());
            return false;
        }

        nlohmann::json j;
        file >> j;

        for (const auto &region_json : j["regions"])
        {
            Region region;
            region.id = region_json["id"].get<std::string>();
            region.type = region_json["type"].get<int>();

            for (const auto &p : region_json["points"])
            {
                geometry_msgs::msg::Point pt;
                pt.x = p["x"].get<double>();
                pt.y = p["y"].get<double>();
                pt.z = p["z"].get<double>();
                region.points.push_back(pt);
            }
            region.long_edge = compute_long_edge(region.points);
            regions_.push_back(region);
            RCLCPP_INFO(this->get_logger(), "加载区域成功: %s (顶点数: %zu)",
                        region.id.c_str(), region.points.size());
        }

        return !regions_.empty();
    }

    bool is_in_region(const geometry_msgs::msg::Point &pt, const Region &region)
    {
        bool inside = false;
        size_t n = region.points.size();

        for (size_t i = 0; i < n; ++i)
        {
            const auto &a = region.points[i];
            const auto &b = region.points[(i + 1) % n];

            // 判断点是否在边界上
            if ((pt.x == a.x && pt.y == a.y) ||
                (pt.x == b.x && pt.y == b.y))
                return true;

            if (((a.y > pt.y) != (b.y > pt.y)) &&
                (pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    int get_region_int(const geometry_msgs::msg::Point &pt, Region &region)
    {
        for (const auto &region_ : regions_)
        {
            if (is_in_region(pt, region_))
            {
                region = region_;
                return region.type;
            }
        }
        return -1;
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(map_frame_id_, base_link_frame_id_, tf2::TimePointZero);
            curr_pos_.x = tf.transform.translation.x;
            curr_pos_.y = tf.transform.translation.y;
            curr_pos_.z = tf.transform.translation.z;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", e.what());
            return;
        }
        Region region_;
        Eigen::Vector2d curr_pos_vect(curr_pos_.x, curr_pos_.y);
        int region_int = get_region_int(curr_pos_, region_);
        auto msg = std_msgs::msg::Int32();
        msg.data = region_int;
        region_int_pub_->publish(msg);

        region_cmd_.chassis_mode = 2;
        region_cmd_.pass_special_region = 0;
        region_cmd_.pass_region_angle = 0.0;

        /* 确保每次进区域都只发一个方向 */
        if (region_int == -1)
        {
            in_region_ = false;
            dir = Eigen::Vector2d::Zero();

            gimbal_cmd_pub_->publish(region_cmd_);
            return;
        }
        // 从没进到进
        if (!in_region_)
        {
            in_region_ = true;
            switch (region_int)
            {
            // 颠簸路段
            case 1:
            {
                Eigen::Vector2d A = region_.long_edge.start;
                Eigen::Vector2d B = region_.long_edge.end;
                // A 更接近 B
                if ((curr_pos_vect - A).squaredNorm() < (curr_pos_vect - B).squaredNorm())
                {
                    // curr 更靠近 A → 发布 A → B 的方向
                    dir = (B - A).normalized();
                    publish_rectangle_debug_marker(region_, dir, curr_pos_);
                }
                else
                {
                    // curr 更靠近 B → 发布 B → A 的方向
                    dir = (A - B).normalized();
                    publish_rectangle_debug_marker(region_, dir, curr_pos_);
                }
                
                break;
            }
            default:
                break;
            }
            tf2::Quaternion q(
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w);
            double _, robot_yaw;
            tf2::Matrix3x3(q).getRPY(_, _, robot_yaw);
            double dir_yaw = std::atan2(dir.y(), dir.x());
    
            double angle = dir_yaw - robot_yaw;
    
            angle = std::fmod(angle + M_PI, 2 * M_PI);
            if (angle<0)
            {
               angle += 2.0*M_PI;
            }
            angle -= M_PI;
            
            
            in_region_angle = angle * 180.0 / M_PI;
            
        }

        region_cmd_.chassis_mode = 1;
        region_cmd_.pass_special_region = 1;
        region_cmd_.pass_region_angle = in_region_angle;
        gimbal_cmd_pub_->publish(region_cmd_);

    }

    void publish_rectangle_debug_marker(
        const Region &region,
        const Eigen::Vector2d &dir,
        const geometry_msgs::msg::Point &p)
    {
        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = map_frame_id_;
        mk.header.stamp = now();
        mk.ns = "rect_debug";
        mk.id = 1;
        mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.scale.x = 0.05;
        mk.color.r = 1.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;
        mk.color.a = 1.0;

        for (auto &pt : region.points)
            mk.points.push_back(pt);
        mk.points.push_back(region.points[0]);
        arrow_pub->publish(mk);

        // ====================
        //     画方向箭头
        // ====================
        visualization_msgs::msg::Marker arrow;
        arrow.header = mk.header;
        arrow.ns = "rect_arrow";
        arrow.id = 2;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.scale.x = 0.1;
        arrow.scale.y = 0.2;
        arrow.scale.z = 0.2;
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        geometry_msgs::msg::Point start = p;
        geometry_msgs::msg::Point end;
        end.x = start.x + dir.x();
        end.y = start.y + dir.y();
        end.z = start.z;

        arrow.points.push_back(start);
        arrow.points.push_back(end);

        arrow_pub->publish(arrow);

        // ====================
        //     点 P 的球
        // ====================
        visualization_msgs::msg::Marker sphere;
        sphere.header = mk.header;
        sphere.id = 3;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15;
        sphere.color.r = 0;
        sphere.color.g = 0;
        sphere.color.b = 1;
        sphere.color.a = 1;
        sphere.pose.position = p;

        arrow_pub->publish(sphere);
    }

    LongEdge compute_long_edge(const std::vector<geometry_msgs::msg::Point> &pts)
    {
        if (pts.size() != 4)
        {
            return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
        }

        Eigen::Vector2d A(pts[0].x, pts[0].y);
        Eigen::Vector2d B(pts[1].x, pts[1].y);
        Eigen::Vector2d C(pts[2].x, pts[2].y);
        Eigen::Vector2d D(pts[3].x, pts[3].y);

        std::array<LongEdge, 4> edges = {
            LongEdge{A, B},
            LongEdge{B, C},
            LongEdge{C, D},
            LongEdge{D, A}};

        double max_len = -1.0;
        LongEdge max_edge;

        for (const auto &e : edges)
        {
            double len = (e.end - e.start).norm();
            if (len > max_len)
            {
                max_len = len;
                max_edge = e;
            }
        }

        return max_edge;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionBehaviorNode>());
    rclcpp::shutdown();
    return 0;
}
