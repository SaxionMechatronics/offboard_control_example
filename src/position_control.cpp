#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

#define NANF nanf("1")

struct Setpoint {
	Setpoint(float x_, float y_, float z_, float yaw_, float duration_){
		x = x_;
		y = y_;
		z = z_;
		yaw = yaw_;

		duration = duration_;
	}

	float x, y, z, yaw, duration;
};

class position_control : public rclcpp::Node {
public:
	position_control() : Node("position_control") {
		// Read config
		readConfig();

		// Setup publishers
		offboardControlModePublisher =
			this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", 10);
		trajectorySetpointPublisher =
			this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
		vehicleCommandPublisher =
			this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);

		// Counter to send 10 setpoints before arming
		offboardSetpointCounter = 0;

		// Set the time when the next setpoint should be send
		tNext = setpoints[0].duration;

		// Send offboard control mode (Not flight mode) and setpoints at a rate of commandRate
		commandTimer = this->create_wall_timer(1s/commandRate, [=]() {sendCommands();});

		displaySetpoint();
	}

	void sendCommands();
	void arm();
	void disarm();

private:
	// Rate at which commands are send to px4
	double commandRate = 10.0;

	unsigned int setpointIndex = 0;

	std::vector<Setpoint> setpoints;

	// Timing to know when the next setpoint should be set
	double tNext;

	rclcpp::TimerBase::SharedPtr commandTimer;
	rclcpp::TimerBase::SharedPtr setpointTimer;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboardControlModePublisher;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectorySetpointPublisher;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicleCommandPublisher;

	uint64_t offboardSetpointCounter;	// counter for the number of setpoints sent

	void readConfig();
	void publishOffboardControlMode();
	void publishTrajectorySetpoint();
	void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void displaySetpoint();
};

void position_control::readConfig(){
	RCLCPP_INFO(this->get_logger(), "Reading setpoints from config");

	bool succes = true;

	rclcpp::Parameter param;

	int i = 0;

	// Read setpoints sp0, sp1, sp.. till the end is reached.
	while (succes){
		this->declare_parameter("sp" + std::to_string(i));
		succes = this->get_parameter("sp" + std::to_string(i), param);

		if (succes) {
			std::vector<double> sp = param.as_double_array();

			if (sp.size() == 5) {
				setpoints.push_back(Setpoint(sp[0], sp[1], sp[2], sp[3], sp[4]));
			}
			else {
				RCLCPP_INFO(this->get_logger(), "Setpoint sp%i is in valid, size: ", sp.size());
			}

			i++;
		}
	}
}

void position_control::sendCommands() {
	// Calculate time remaining till next setpoint
	tNext -= 1.0/commandRate;

	// Go to the next setpoint if needed
	if (tNext < 0.0){
		setpointIndex++;

		if (setpointIndex < setpoints.size()){
			tNext = setpoints[setpointIndex].duration;

			displaySetpoint();
		}
	}

	if (offboardSetpointCounter == 10) {
		// Change to Offboard flight mode after 10 setpoints have been send
		this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

		// Arm the vehicle
		this->arm();
	}

	// Offboard_control_mode needs to be paired with trajectory_setpoint
	publishOffboardControlMode();
	publishTrajectorySetpoint();

	// Stop the counter after reaching 11, so setting offboard mode and arming only happens once
	if (offboardSetpointCounter < 11) {
		offboardSetpointCounter++;
	}
}

void position_control::arm() {
	publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void position_control::disarm() {
	publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO_ONCE(this->get_logger(), "Sending disarm commands");
}

void position_control::publishOffboardControlMode() {
	// Publish a message to tell PX4 which offboard control mode it should use (type of setpoint, not flight mode)
	OffboardControlMode msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboardControlModePublisher->publish(msg);
}

void position_control::publishTrajectorySetpoint() {
	// Check if any checkpoints remain
	if (setpointIndex < setpoints.size()){
		// Create a setpoints message to publish to PX4 and set the values
		TrajectorySetpoint msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		const Setpoint sp = setpoints[setpointIndex];

		msg.position[0] = sp.x;
		msg.position[1] = sp.y;
		msg.position[2] = sp.z;
		msg.yaw = sp.yaw;

		// Publish the message
		trajectorySetpointPublisher->publish(msg);
	}
	else {
		// If no setpoints remain -> disarm
		disarm();
	}
}

void position_control::publishVehicleCommand(uint16_t command, float param1, float param2) {
	VehicleCommand msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicleCommandPublisher->publish(msg);
}

void position_control::displaySetpoint(){
	Setpoint &sp = setpoints[setpointIndex];

	RCLCPP_INFO(this->get_logger(), "\nPosition: \n X:   %.2f\n Y:   %.2f\n Z:   %.2f\n Yaw: %.2f\n", sp.x, sp.y, sp.z, sp.yaw);
}

int main(int argc, char* argv[]) {
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	// Start spinning the ROS2 Node
	rclcpp::spin(std::make_shared<position_control>());

	rclcpp::shutdown();
	return 0;
}
