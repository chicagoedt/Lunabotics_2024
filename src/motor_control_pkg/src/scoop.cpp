#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/duty_cycle_output.hpp"
#include "SparkMax.hpp"

class ScoopNode : public rclcpp::Node
{
	public:
		ScoopNode() : Node("scoop"){
			duty_subscriber = this->create_subscription<interfaces_pkg::msg::DutyCycleOutput>(
     			 "control/duty_cycle", 10, std::bind(&ScoopNode::duty_cycle_control, this, std::placeholders::_1));

	SparkMax left_lift("can0", 3);
	//Creates left_lift object

	left_lift.SetIdleMode(IdleMode::kBrake);
	left_lift.SetMotorType(MotorType::kBrushed);
	left_lift.SetHardLimitFwdEn(false);
	left_lift.SetHardLimitFwdEn(false);
	left_lift.BurnFlash();

	}

	private:
		void duty_cycle_control(const interfaces_pkg::msg::DutyCycleOutput & msg) const {

			RCLCPP_INFO(this->get_logger(), "%f %f %f %f", msg.left_lift_output, msg.right_lift_output, msg.tilt_output, msg.vibrator_output);

			SparkMax left_lift("can0", 3);
     			//Creates left_lift object
			left_lift.Heartbeat();
      			left_lift.SetDutyCycle(msg.left_lift_output);
      			//Sets left lift output
      			RCLCPP_INFO(this->get_logger(), "SparkMax::Heartbeat(); left_lift.SetDutyCycle(msg.left_lift_output);");
		}

		rclcpp::Subscription<interfaces_pkg::msg::DutyCycleOutput>::SharedPtr duty_subscriber;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScoopNode>());
  rclcpp::shutdown();
  return 0;
}
