#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/duty_cycle_output.hpp"
#include "SparkMax.hpp"

class ScoopNode : public rclcpp::Node
{
	public:
		ScoopNode()
		: Node("scoop"){
			duty_subscriber = this->create_subscription<interfaces_pkg::msg::DutyCycleOutput>(
      "control/duty_cycle", 10, std::bind(&ScoopNode::duty_cycle_control, this, std::placeholders::_1));
      
      /*
      SparkMax left_lift("can0", 3);
      //Creates left_lift object
      
      left_lift.SetIdleMode(IdleMode::kBrake);
      left_lift.SetMotorType(MotorType::kBrushed);
      left_lift.SetInverted(true);  //TO-DO: test out actuators to see if they are inverted or not
      left_lift.BurnFlash();
      */
      
		}
		
	private:
		void duty_cycle_control(const interfaces_pkg::msg::DutyCycleOutput & msg) const {
		
			RCLCPP_INFO(this->get_logger(), "%f %f %f %f", msg.left_lift_output, msg.right_lift_output, msg.tilt_output, msg.vibrator_output);
			
			/*
			SparkMax left_lift("can0", 3);
     			//Creates left_lift object
			SparkMax::Heartbeat();
      			left_lift.SetDutyCycle(msg.left_lift_output);
      			*/
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
