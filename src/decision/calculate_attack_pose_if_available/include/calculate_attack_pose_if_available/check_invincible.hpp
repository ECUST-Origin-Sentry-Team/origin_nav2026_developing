#ifndef CHECK_INVINCIBLE_HPP
#define CHECK_INVINCIBLE_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "referee_msg/msg/referee.hpp"
#include "rm_interfaces/srv/invincible_inquire.hpp"
#include <unordered_map>

enum class InvincibleState
{
    NORMAL = 0,
    DEAD,
    REVIVING,
    COIN_REVIVING
};

static const std::unordered_map<std::string, uint16_t referee_msg::msg::Referee::*> hp_map = {
    {"red_1_hp", &referee_msg::msg::Referee::red_1_hp},
    {"red_2_hp", &referee_msg::msg::Referee::red_2_hp},
    {"red_3_hp", &referee_msg::msg::Referee::red_3_hp},
    {"red_4_hp", &referee_msg::msg::Referee::red_4_hp},
    {"red_7_hp", &referee_msg::msg::Referee::red_7_hp},
    {"blue_1_hp", &referee_msg::msg::Referee::blue_1_hp},
    {"blue_2_hp", &referee_msg::msg::Referee::blue_2_hp},
    {"blue_3_hp", &referee_msg::msg::Referee::blue_3_hp},
    {"blue_4_hp", &referee_msg::msg::Referee::blue_4_hp},
    {"blue_7_hp", &referee_msg::msg::Referee::blue_7_hp},
};

struct EnemyInfo
{
    std::string enemy_name;
    InvincibleState enemy_state;
    int enemy_reviving_time;
    int enemy_hp;
};

class InvincibleChecker : public rclcpp::Node
{
public:
    InvincibleChecker();
    ~InvincibleChecker();

    rclcpp::Subscription<referee_msg::msg::Referee>::SharedPtr sub_referee_;
    rclcpp::Service<rm_interfaces::srv::InvincibleInquire>::SharedPtr inv_checker_srv_;

private:
    void update_enemy_status(const referee_msg::msg::Referee &msg, EnemyInfo &enemy, int invincible_time);
    int get_enemy_hp(const referee_msg::msg::Referee &msg,
                     const std::string &color,
                     const std::string &enemy_name);
    void referee_callback(const referee_msg::msg::Referee::SharedPtr msg);
    void get_enemy_invincible_status(const rm_interfaces::srv::InvincibleInquire::Request::SharedPtr request,
                                     rm_interfaces::srv::InvincibleInquire::Response::SharedPtr response);

    std::unordered_map<std::string, EnemyInfo> enemy_info;
    std::string self_color = "red";
    std::string their_color = "blue";


};

#endif