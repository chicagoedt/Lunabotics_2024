#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/duty_cycle_output.hpp"
#include "SparkMax.hpp"

class DrivetrainNode : public rclcpp::Node
{
	public:
		DrivetrainNode()
		: Node("drivetrain") {
			duty_subscriber = this->create_subscription<interfaces_pkg::msg::DutyCycleOutput>(
      "control/duty_cycle", 10, std::bind(&DrivetrainNode::duty_cycle_control, this, std::placeholders::_1));
      
      /*
      SparkMax left_drive("can0", 1);
      SparkMax right_drive("can0", 2);
      //Creates SparkMax Objects
      
      left_drive.SetIdleMode(IdleMode::kCoast);
      left_drive.SetMotorType(MotorType::kBrushless);
      left_drive.BurnFlash();
      //Burn left drive motor
      
      right_drive.SetIdleMode(IdleMode::kCoast);
      right_drive.SetMotorType(MotorType::kBrushless);
      right_drive.BurnFlash();
      //Burn right drive motor
      */
		}
		
	private:
		void duty_cycle_control(const interfaces_pkg::msg::DutyCycleOutput & msg) const {
			
			RCLCPP_INFO(this->get_logger(), "%f %f", msg.left_drive_output, msg.right_drive_output);
			/*
			SparkMax left_drive("can0", 1);
      			SparkMax right_drive("can0", 2);
      			//Creates SparkMax Objects
      			
			SparkMax::Heartbeat();
			left_drive.SetDutyCycle(msg.left_drive_output);
			right_drive.SetDutyCycle(msg.right_drive_output);
			//send duty cycle output to left and right drive
			
			*/
			
			RCLCPP_INFO(this->get_logger(), "SparkMax::Heartbeat(); left_drive.SetDutyCycle(msg.left_drive_output); right_drive.SetDutyCycle(msg.right_drive_output);");
			
		}
		
		rclcpp::Subscription<interfaces_pkg::msg::DutyCycleOutput>::SharedPtr duty_subscriber;
		
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrivetrainNode>());
  rclcpp::shutdown();
  return 0;
}
