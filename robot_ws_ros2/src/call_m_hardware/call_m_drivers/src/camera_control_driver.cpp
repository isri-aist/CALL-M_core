#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define BROADCAST_ID 254

class CameraControlNode : public rclcpp::Node {
public:
  CameraControlNode() : Node("camera_control_driver_node") {
    RCLCPP_INFO(this->get_logger(), "camera control driver node");

    // Open Serial Port
    initialize_params();
    refresh_params();
    this->portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    this->dxl_comm_result = this->portHandler->openPort();
    if (this->dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("camera_control_driver_node"), "Failed to open the port: %s",device_name.c_str());
      
    } else {
      RCLCPP_INFO(rclcpp::get_logger("camera_control_driver_node"), "Succeeded to open the port: %s",device_name.c_str());
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    this->dxl_comm_result = this->portHandler->setBaudRate(BAUDRATE);
    if (this->dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("camera_control_driver_node"), "Failed to set the baudrate!");
      
    } else {
      RCLCPP_INFO(rclcpp::get_logger("camera_control_driver_node"), "Succeeded to set the baudrate.");
    }

    this->setupDynamixel(BROADCAST_ID);

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    //Initialise positions
    int id1 = 2;
    int id2 = 3; 
    int init_pos = 2000;
    uint8_t dxl_error_init= 0;
    uint32_t goal_position = (unsigned int)init_pos;  // Convert int32 -> uint32
    packetHandler->write4ByteTxRx(portHandler, (uint8_t)id1, ADDR_GOAL_POSITION, goal_position, &dxl_error_init);
    dxl_error_init= 0;
    packetHandler->write4ByteTxRx(portHandler, (uint8_t)id2, ADDR_GOAL_POSITION, goal_position, &dxl_error_init);

    set_position_subscriber_ =
        this->create_subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
            "set_position", QOS_RKL10V,
            [this](const dynamixel_sdk_custom_interfaces::msg::SetPosition::SharedPtr msg) -> void {
              uint8_t dxl_error = 0;

              // Position Value of X series is 4 byte data.
              // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
              uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

              // Write Goal Position (length : 4 bytes)
              // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
              dxl_comm_result = packetHandler->write4ByteTxRx(
                  portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, goal_position, &dxl_error);

              if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
              } else if (dxl_error != 0) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
              } else {
                RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id,
                            msg->position);
              }
            });

    auto get_present_position = [this](
                                     const std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request> request,
                                     std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response>
                                         response) -> void {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
          portHandler, (uint8_t)request->id, ADDR_PRESENT_POSITION,
          reinterpret_cast<uint32_t *>(&present_position), &dxl_error);

      RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present Position: %d]", request->id, present_position);

      response->position = present_position;
    };

    get_position_server_ =
        this->create_service<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position",
                                                                                get_present_position);
  }

  ~CameraControlNode() {}

  void setupDynamixel(uint8_t dxl_id) {
    // Use Position Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 3, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("camera_control_driver_node"), "Failed to set Position Control Mode.");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("camera_control_driver_node"), "Succeeded to set Position Control Mode.");
    }

    // Enable Torque of DYNAMIXEL
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("camera_control_driver_node"), "Failed to enable torque.");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("camera_control_driver_node"), "Succeeded to enable torque.");
    }
  }

  void initialize_params(){
      this->declare_parameter("device_name","/dev/ttyUSB0"); 
  }

  void refresh_params(){
      this->device_name=get_parameter("device_name").as_string();
  }

  rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr get_position_server_;

  std::string device_name;

  uint32_t present_position = 0;

  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;

  uint8_t dxl_error = 0;
  uint32_t goal_position = 0;
  int dxl_comm_result = COMM_TX_FAIL;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  node->packetHandler->write1ByteTxRx(node->portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0, &node->dxl_error);

  return 0;
}
