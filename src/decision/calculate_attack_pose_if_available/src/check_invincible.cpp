#include "calculate_attack_pose_if_available/check_invincible.hpp"

InvincibleChecker::InvincibleChecker()
    : Node("invincible_checker")
{
    this->declare_parameter("self_color", "red");
    this->get_parameter("self_color", self_color);
    their_color = (self_color == "red") ? "blue" : "red" ;
    // 初始化 enemy_info_
    enemy_info["1"] = {"1_hp", InvincibleState::NORMAL, 0, 100};
    enemy_info["2"] = {"2_hp", InvincibleState::NORMAL, 0, 100};
    enemy_info["3"] = {"3_hp", InvincibleState::NORMAL, 0, 100};
    enemy_info["4"] = {"4_hp", InvincibleState::NORMAL, 0, 100};
    enemy_info["sentry"] = {"7_hp", InvincibleState::NORMAL, 0, 100};
    sub_referee_ = this->create_subscription<referee_msg::msg::Referee>(
        "/Referee",
        rclcpp::QoS(10),
        std::bind(&InvincibleChecker::referee_callback, this, std::placeholders::_1));
    inv_checker_srv_ = this->create_service<rm_interfaces::srv::InvincibleInquire>(
        "/enemy_invincible_status",
        std::bind(&InvincibleChecker::get_enemy_invincible_status, this, std::placeholders::_1, std::placeholders::_2));
    std::cout<<"their_color:  "<<their_color<<std::endl;
}

InvincibleChecker::~InvincibleChecker()
{
}

int InvincibleChecker::get_enemy_hp(const referee_msg::msg::Referee &msg,
                                    const std::string &color,
                                    const std::string &enemy_name)
{
    std::string key = color + "_" + enemy_name;
    auto it = hp_map.find(key);
    if (it != hp_map.end())
    {
        return msg.*(it->second);
    }
    RCLCPP_WARN(rclcpp::get_logger("hp_reader"), "Unknown key: %s", key.c_str());
    return -1;
}

void InvincibleChecker::update_enemy_status(const referee_msg::msg::Referee &msg, EnemyInfo &enemy, int invincible_time)
{
    int enemy_hp = get_enemy_hp(msg, their_color, enemy.enemy_name);
    int stage_remain_time = msg.stage_remain_time;

    switch (enemy.enemy_state)
    {
    case InvincibleState::NORMAL:
        if (enemy_hp == 0)
        {
            enemy.enemy_state = InvincibleState::DEAD;
            enemy.enemy_reviving_time = 0;
        }
        else
        {
            enemy.enemy_state = InvincibleState::NORMAL;
            enemy.enemy_reviving_time = 0;
        }
        break;

    case InvincibleState::DEAD:
        if (enemy_hp > 0 && enemy_hp < 100)
        {
            enemy.enemy_state = InvincibleState::REVIVING;
            enemy.enemy_reviving_time = stage_remain_time - invincible_time;
        }
        else if (enemy_hp >= 100)
        {
            enemy.enemy_state = InvincibleState::COIN_REVIVING;
            enemy.enemy_reviving_time = stage_remain_time - 3;
        }
        else
        {
            enemy.enemy_state = InvincibleState::DEAD;
            enemy.enemy_reviving_time = 0;
        }
        break;

    case InvincibleState::REVIVING:
        if (stage_remain_time <= enemy.enemy_reviving_time || enemy_hp > enemy.enemy_hp)
        { // 可以击打
            enemy.enemy_state = InvincibleState::NORMAL;
            enemy.enemy_reviving_time = 0;
        }
        else
        {
            enemy.enemy_state = InvincibleState::REVIVING;
        }
        break;

    case InvincibleState::COIN_REVIVING:
        if (stage_remain_time <= enemy.enemy_reviving_time)
        {
            enemy.enemy_state = InvincibleState::NORMAL;
            enemy.enemy_reviving_time = 0;
        }
        else
        {
            enemy.enemy_state = InvincibleState::COIN_REVIVING;
        }
        break;
    }

    enemy.enemy_hp = enemy_hp;
}

void InvincibleChecker::referee_callback(const referee_msg::msg::Referee::SharedPtr msg)
{

    if (msg->game_progress != 4)
    {
        return;
    }

    for (auto &pair : enemy_info)
    {
        const std::string &key = pair.first;
        auto &value = pair.second;
        if (key == "sentry")
        {
            update_enemy_status(*msg, value, 60);
        }
        else
        {
            update_enemy_status(*msg, value, 10);
        }
    }
}

void InvincibleChecker::get_enemy_invincible_status(const rm_interfaces::srv::InvincibleInquire::Request::SharedPtr request,
                                                    rm_interfaces::srv::InvincibleInquire::Response::SharedPtr response)
{
    std::string enemy_name = request->enemy_id;
    auto it = enemy_info.find(enemy_name);
    if (it != enemy_info.end())
    {
        if (it->second.enemy_state == InvincibleState::NORMAL)
        {
            response->attack_available = true;
        }
        else
        {
            response->attack_available = false;

        }
    }
    else
    {
        response->attack_available = true;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InvincibleChecker>());
    rclcpp::shutdown();
    return 0;
}