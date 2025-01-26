//aakash bajaj
// 1/26/2025
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "SparkMax.hpp"
#include "lunabot_msgs/action/excavation.hpp"

class ExcavationNode : public rclcpp::Node
{
public:
    using Excavation = lunabot_msgs::action::Excavation;
    using GoalHandleExcavation = rclcpp_action::ServerGoalHandle<Excavation>;

    ExcavationNode()
        : Node("excavation_node"),
          current_position_(0.355),
          target_position_(4.0),
          goal_active_(false)
    {
        action_server_ = rclcpp_action::create_server<Excavation>(
            this,
            "excavation_action",
            std::bind(&ExcavationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExcavationNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ExcavationNode::handle_accepted, this, std::placeholders::_1));

        left_lift_ = std::make_shared<SparkMax>("can0", 3); // actuator has id 3
        left_lift_->SetIdleMode(IdleMode::kBrake);
        left_lift_->SetMotorType(MotorType::kBrushed);
        left_lift_->SetInverted(true); // inversion
        left_lift_->BurnFlash();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ExcavationNode::update_position, this));

        RCLCPP_INFO(this->get_logger(), "ExcavationNode initialized.");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Excavation::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received excavation goal request.");
        (void)uuid;
        if (goal_active_)
        {
            RCLCPP_WARN(this->get_logger(), "A goal is already active. Rejecting new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received excavation cancel request.");
        goal_active_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        std::thread{std::bind(&ExcavationNode::execute_goal, this, goal_handle)}.detach();
    }

    void execute_goal(const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing excavation goal...");
        goal_active_ = true;

        auto feedback = std::make_shared<Excavation::Feedback>();
        auto result = std::make_shared<Excavation::Result>();

        left_lift_->SetDutyCycle(0.5); 

        while (rclcpp::ok() && goal_active_)
        {
            double simulated_increment = 0.05; 
            current_position_ += simulated_increment;

            feedback->current_position = current_position_;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Current position: %.2f", current_position_);

            if (current_position_ >= target_position_)
            {
                RCLCPP_INFO(this->get_logger(), "Excavation complete. Target position reached.");
                left_lift_->SetDutyCycle(0.0); 
                result->success = true;
                goal_handle->succeed(result);
                goal_active_ = false;
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!goal_active_)
        {
            RCLCPP_INFO(this->get_logger(), "Excavation goal canceled.");
            result->success = false;
            goal_handle->canceled(result);
        }
    }

    void update_position()
    {
        if (!goal_active_)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Actuator idle at position %.2f.", current_position_);
        }
    }

    rclcpp_action::Server<Excavation>::SharedPtr action_server_;
    std::shared_ptr<SparkMax> left_lift_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_position_;
    const double target_position_;
    bool goal_active_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcavationNode>());
    rclcpp::shutdown();
    return 0;
}
