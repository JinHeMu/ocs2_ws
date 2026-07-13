#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dh_gripper_driver/msg/gripper_ctrl.hpp"
#include "dh_gripper_driver/msg/gripper_state.hpp"
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>

class DhAg95GripperDriver : public rclcpp::Node
{
	public:
	DhAg95GripperDriver() : Node("dh_ag95_gripper_driver")
	{
		// 声明参数
		this->declare_parameter("device_port", "/dev/ttyUSB0");
		this->declare_parameter("baudrate", 115200);
		this->declare_parameter("gripper_id", 1);
		this->declare_parameter("max_position", 100.0);  // AG95最大开口100mm
		this->declare_parameter("max_force", 100.0); // AG95最大夹持力100N
		this->declare_parameter("min_position", 0.0);// AG95最小开口0mm
		this->declare_parameter("default_speed", 50.0);  // 默认速度50%
		this->declare_parameter("auto_init", true);  // 自动初始化参数
		this->declare_parameter("force_initialization", false);  // 强制初始化参数

		// 获取参数
		device_port_ = this->get_parameter("device_port").as_string();
		baudrate_ = this->get_parameter("baudrate").as_int();
		gripper_id_ = this->get_parameter("gripper_id").as_int();
		max_position_ = this->get_parameter("max_position").as_double();
		max_force_ = this->get_parameter("max_force").as_double();
		min_position_ = this->get_parameter("min_position").as_double();
		default_speed_ = this->get_parameter("default_speed").as_double();
		auto_init_ = this->get_parameter("auto_init").as_bool();
		force_init_ = this->get_parameter("force_initialization").as_bool();

		// 初始化串口
		serial_fd_ = -1;
		is_initialized_by_driver_ = false;  // 驱动层初始化状态标志

		// DH夹爪控制接口
		gripper_ctrl_sub_ = this->create_subscription<dh_gripper_driver::msg::GripperCtrl>(
		"gripper/ctrl", 1,
		std::bind(&DhAg95GripperDriver::gripperCtrlCallback, this, std::placeholders::_1));

		gripper_state_pub_ = this->create_publisher<dh_gripper_driver::msg::GripperState>(
		"gripper/state", 10);

		// Joint状态发布
		joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
		"joint_states", 10);

		// 定时器 - 20Hz状态更新
		timer_ = this->create_wall_timer(
		std::chrono::milliseconds(50),
		std::bind(&DhAg95GripperDriver::timerCallback, this));

		// 初始化串口连接
		initializeSerial();
		
		RCLCPP_INFO(this->get_logger(), "DH AG95 Gripper Driver initialized");
		RCLCPP_INFO(this->get_logger(), "Device: %s, Baudrate: %d", device_port_.c_str(), baudrate_);
		RCLCPP_INFO(this->get_logger(), "Position range: %.1f - %.1f mm", min_position_, max_position_);
		RCLCPP_INFO(this->get_logger(), "Auto-initialization: %s", auto_init_ ? "enabled" : "disabled");

		// 自动初始化夹爪
		if (auto_init_ && is_connected_) {
			RCLCPP_INFO(this->get_logger(), "Auto-initializing gripper...");
			initializeGripper();
		}
	}

	~DhAg95GripperDriver()
	{
		if (serial_fd_ >= 0) {
			close(serial_fd_);
		}
	}

	private:
	void gripperCtrlCallback(const dh_gripper_driver::msg::GripperCtrl::SharedPtr msg)
	{
		if (msg->initialize) {
			initializeGripper();
			// 不要直接返回，继续执行位置控制
		}

		// 验证参数范围
		float position = std::clamp(msg->position, (float)min_position_, (float)max_position_);
		float force = std::clamp(msg->force, 20.0f, (float)max_force_);  // DH官方：20-100%
		float speed = std::clamp(msg->speed, 1.0f, 100.0f);// DH官方：1-100%

		if (position != msg->position) {
			RCLCPP_WARN(this->get_logger(), "Position clamped from %.2f to %.2f", msg->position, position);
		}

		// 发送位置和力控命令到DH夹爪
		// 如果是初始化命令，跳过初始化检查
		sendGripperCommand(position, force, speed, msg->initialize);
	}

	void timerCallback()
	{
		// 读取夹爪状态
		readGripperState();

		// 发布Joint状态
		publishJointState();

		// 发布夹爪状态
		publishGripperState();
	}

	void initializeSerial()
	{
		RCLCPP_INFO(this->get_logger(), "Initializing serial connection to %s", device_port_.c_str());

		serial_fd_ = open(device_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

		if (serial_fd_ < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", device_port_.c_str());
			is_connected_ = false;
			return;
		}

		// 配置串口参数
		struct termios options;
		tcgetattr(serial_fd_, &options);

		// 设置波特率
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);

		// 8N1
		options.c_cflag &= ~PARENB;// 无奇偶校验
		options.c_cflag &= ~CSTOPB;// 1个停止位
		options.c_cflag &= ~CSIZE; // 清除数据位设置
		options.c_cflag |= CS8;// 8个数据位

		// 无流控制
		options.c_cflag &= ~CRTSCTS;

		// 启用接收器，设置本地模式
		options.c_cflag |= (CLOCAL | CREAD);

		// 原始模式
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		options.c_iflag &= ~(IXON | IXOFF | IXANY);
		options.c_oflag &= ~OPOST;

		// 设置超时
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;  // 1秒超时

		tcsetattr(serial_fd_, TCSANOW, &options);

		is_connected_ = true;

		// 初始化夹爪状态
		gripper_state_.header.frame_id = "gripper_base_link";
		gripper_state_.is_initialized = false;
		gripper_state_.grip_state = 0;  // 未初始化
		gripper_state_.position = 0.0;
		gripper_state_.target_position = 0.0;
		gripper_state_.target_force = 0.0;
		gripper_state_.current_force = 0.0;
		gripper_state_.object_detected = false;

		RCLCPP_INFO(this->get_logger(), "Serial connection established");
	}

	void initializeGripper()
	{
		if (!is_connected_) {
			RCLCPP_WARN(this->get_logger(), "Cannot initialize gripper: not connected");
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Initializing gripper...");

		bool init_success = false;

		// 读取当前初始化状态
		int current_init_state = 0;
		if (readRegister(0x0200, current_init_state)) {
			RCLCPP_INFO(this->get_logger(), "Current initialization state: %d", current_init_state);
			if (current_init_state == 1) {
				RCLCPP_INFO(this->get_logger(), "Gripper already initialized");
				init_success = true;
				is_initialized_by_driver_ = true;
				gripper_state_.is_initialized = true;
			}
		}

		// 初始化命令
		int init_waitCount = 0;
		if(!init_success) {
			if(writeRegister(0x0100, 0xA5)) {
				while (!init_success) {
					RCLCPP_INFO(this->get_logger(), "Initialization command sent, waiting for completion...");
					usleep(3000000);  // 等待3秒

					// 检查初始化是否成功
					init_waitCount++;
					int final_init_state = 0;
					if (readRegister(0x0200, final_init_state)) {
						RCLCPP_INFO(this->get_logger(), "Final initialization state: %d", final_init_state);
						if (final_init_state == 1) {
							init_waitCount = 0;
							init_success = true;
							is_initialized_by_driver_ = true;
							gripper_state_.is_initialized = true;
							RCLCPP_INFO(this->get_logger(), "Gripper initialization completed successfully");
						}
					}
					if(init_waitCount > 3) break;
				}
			}
		}

		if(!init_success)
		{
			RCLCPP_ERROR(this->get_logger(), "Initialization failed");
			is_initialized_by_driver_ = false;
			gripper_state_.is_initialized = false;
			return ;
		}

		// 检查夹爪是否能响应基本命令
		int test_pos = 0;
		if (readRegister(0x0202, test_pos)) {
			RCLCPP_INFO(this->get_logger(), "Communication test successful, current position: %d", test_pos);
			// 如果能读取位置，说明通信正常，可能夹爪已经可以工作
			RCLCPP_INFO(this->get_logger(), "Gripper ready, waiting for the control");
		}
	}

	void sendGripperCommand(float position, float force, float speed, bool skip_init_check = false)
	{
		if (!is_connected_) {
			RCLCPP_WARN(this->get_logger(), "Gripper not connected");
			return;
		}

		// 如果驱动层已初始化，跳过初始化检查
		if (!skip_init_check && !is_initialized_by_driver_) {
			RCLCPP_WARN(this->get_logger(), "Gripper not initialized. Attempting to initialize now...");
			initializeGripper();
			if (!is_initialized_by_driver_) {
				RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper");
				return;
			}
		}

		RCLCPP_INFO(this->get_logger(), "Sending command: pos=%.2f, force=%.2f, speed=%.2f", position, force, speed);

		// 转换参数为DH协议格式
		// 位置：0-100mm -> 0-1000 (DH官方格式)
		int pos_value = static_cast<int>((position / max_position_) * 1000);
		int force_value = static_cast<int>(force);// 20-100%
		int speed_value = static_cast<int>(speed);// 1-100%

		RCLCPP_INFO(this->get_logger(), "Converted values: pos=%d, force=%d, speed=%d", pos_value, force_value, speed_value);

		// 发送命令到DH夹爪，分别记录每个寄存器的结果
		bool pos_success = false;
		bool force_success = false;
		bool speed_success = false;

		RCLCPP_INFO(this->get_logger(), "Writing position register 0x0103 = %d", pos_value);
		pos_success = writeRegister(0x0103, pos_value);   // 目标位置

		RCLCPP_INFO(this->get_logger(), "Writing force register 0x0101 = %d", force_value);
		force_success = writeRegister(0x0101, force_value); // 目标力

		RCLCPP_INFO(this->get_logger(), "Writing speed register 0x0104 = %d", speed_value);
		speed_success = writeRegister(0x0104, speed_value); // 目标速度

		// 总结结果
		bool overall_success = pos_success && force_success && speed_success;
		RCLCPP_INFO(this->get_logger(), "   Position (0x0103): %s", pos_success ? "SUCCESS" : "FAILED");
		RCLCPP_INFO(this->get_logger(), "   Force (0x0101):%s", force_success ? "SUCCESS" : "FAILED");
		RCLCPP_INFO(this->get_logger(), "   Speed (0x0104):%s", speed_success ? "SUCCESS" : "FAILED");

		if (overall_success) {
			// 更新目标状态
			gripper_state_.target_position = position;
			gripper_state_.target_force = force;
			gripper_state_.grip_state = 0;  // 移动中
			RCLCPP_INFO(this->get_logger(), "All commands sent successfully");
		} else {
			RCLCPP_ERROR(this->get_logger(), "One or more register writes failed");
		}
	}

	void readGripperState()
	{
		if (!is_connected_) return;

		// 读取DH夹爪状态
		int init_state = 0, grip_state = 0, current_pos = 0;

		// 只有在驱动层未初始化时才读取硬件初始化状态
		if (!is_initialized_by_driver_) {
			if (readRegister(0x0200, init_state)) {
				gripper_state_.is_initialized = (init_state == 3);
			}
		} else {
			// 驱动层已初始化，保持初始化状态
			gripper_state_.is_initialized = true;
		}

		if (readRegister(0x0201, grip_state)) {
			gripper_state_.grip_state = grip_state;	
		}

		if (readRegister(0x0202, current_pos)) {
			// 转换位置：0-1000 -> 0-100mm
			gripper_state_.position = (current_pos / 1000.0) * max_position_;
		}

		// 更新时间戳
		gripper_state_.header.stamp = this->now();
	}

	bool writeRegister(int address, int value)
	{
		if (serial_fd_ < 0) return false;

		// DH官方Modbus RTU写寄存器命令（功能码06）
		unsigned char send_buf[8];
		send_buf[0] = gripper_id_;   // 设备地址
		send_buf[1] = 0x06;  // 功能码：写单个寄存器
		send_buf[2] = (address >> 8) & 0xFF; // 寄存器地址高字节
		send_buf[3] = address & 0xFF;// 寄存器地址低字节
		send_buf[4] = (value >> 8) & 0xFF;   // 数据高字节
		send_buf[5] = value & 0xFF;  // 数据低字节

		// 计算CRC
		unsigned short crc = calculateCRC(send_buf, 6);
		send_buf[6] = crc & 0xFF;
		send_buf[7] = (crc >> 8) & 0xFF;

		// 发送命令
		ssize_t bytes_written = write(serial_fd_, send_buf, 8);
		if (bytes_written != 8) {
			return false;
		}

		// 等待响应
		usleep(50000);  // 50ms等待时间

		// 读取响应
		unsigned char recv_buf[8];
		ssize_t bytes_read = read(serial_fd_, recv_buf, 8);

		if (bytes_read == 8) {
			// 验证响应（应该与发送的命令相同）
			for (int i = 0; i < 8; i++) {
				if (send_buf[i] != recv_buf[i]) {
					RCLCPP_ERROR(this->get_logger(), "error byte index = %d, error byte = %d", i, (int)recv_buf[i]);
					return false;
				}
			}
			return true;
		}
		else
			RCLCPP_ERROR(this->get_logger(), "Read byte length = %d", (int)bytes_read);

		return false;
	}

	bool readRegister(int address, int& value)
	{
		if (serial_fd_ < 0) return false;

		// DH官方Modbus RTU读寄存器命令（功能码03）
		unsigned char send_buf[8];
		send_buf[0] = gripper_id_;   // 设备地址
		send_buf[1] = 0x03;  // 功能码：读保持寄存器
		send_buf[2] = (address >> 8) & 0xFF; // 寄存器地址高字节
		send_buf[3] = address & 0xFF;// 寄存器地址低字节
		send_buf[4] = 0x00;  // 寄存器数量高字节
		send_buf[5] = 0x01;  // 寄存器数量低字节（读1个寄存器）

		// 计算CRC
		unsigned short crc = calculateCRC(send_buf, 6);
		send_buf[6] = crc & 0xFF;
		send_buf[7] = (crc >> 8) & 0xFF;

		// 发送命令
		ssize_t bytes_written = write(serial_fd_, send_buf, 8);
		if (bytes_written != 8) {
			return false;
		}

		// 等待响应
		usleep(50000);  // 50ms等待时间

		// 读取响应
		unsigned char recv_buf[7];
		ssize_t bytes_read = read(serial_fd_, recv_buf, 7);

		if (bytes_read == 7) {
			// 验证响应
			if (recv_buf[0] != gripper_id_ || recv_buf[1] != 0x03) {
				return false;
			}	

			// 验证CRC
			unsigned short recv_crc = (recv_buf[6] << 8) | recv_buf[5];
			unsigned short calc_crc = calculateCRC(recv_buf, 5);
			if (recv_crc != calc_crc) {
				return false;
			}

			// 提取数据
			value = (recv_buf[3] << 8) | recv_buf[4];
			return true;
		}

		return false;
	}

	unsigned short calculateCRC(const unsigned char* data, int length)
	{
		// DH官方CRC16计算（使用查表法）
		static const unsigned short crc_table[] = {
			0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
			0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
			0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
			0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
			0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
			0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
			0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
			0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
			0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
			0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
			0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
			0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
			0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
			0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
			0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
			0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
			0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
			0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
			0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
			0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
			0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
			0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
			0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
			0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
			0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
			0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
			0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
			0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
			0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
			0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
			0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
			0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
		};

		unsigned short crc = 0xFFFF;

		for (int i = 0; i < length; i++) {
			unsigned char temp = data[i] ^ crc;
			crc >>= 8;
			crc ^= crc_table[temp];
		}

		return crc;
	}

	void publishJointState()
	{
		auto joint_state = sensor_msgs::msg::JointState();
		joint_state.header.stamp = this->now();
		joint_state.header.frame_id = "base_link";

		// 只发布主`关节
		joint_state.name = {"gripper_finger1_joint"};

		// 计算主关节角度
		// 假设 gripper_state_.position 是当前开口（单位mm），max_position_ 是最大开口（单位mm）
		double closed_angle = -0.94; // 最大合拢角度
		double open_mm = gripper_state_.position; // 当前开口
		double max_mm = max_position_; // 最大开口
		double angle = closed_angle * (max_mm - open_mm) / max_mm; // 合拢=0.93，张开=0

		joint_state.position = {angle};

		joint_state_pub_->publish(joint_state);
	}

	void publishGripperState()
	
	{
		gripper_state_pub_->publish(gripper_state_);
	}

	// 成员变量
	rclcpp::Subscription<dh_gripper_driver::msg::GripperCtrl>::SharedPtr gripper_ctrl_sub_;
	rclcpp::Publisher<dh_gripper_driver::msg::GripperState>::SharedPtr gripper_state_pub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	// 串口相关
	std::string device_port_;
	int baudrate_;
	int gripper_id_;
	int serial_fd_;
	bool is_connected_;

	// 夹爪参数
	double max_position_;
	double max_force_;
	double min_position_;
	double default_speed_;
	bool auto_init_;// 自动初始化参数
	bool force_init_;// 强制初始化参数
	bool is_initialized_by_driver_; // 驱动层初始化状态标志

	// 夹爪状态
	dh_gripper_driver::msg::GripperState gripper_state_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DhAg95GripperDriver>());
	rclcpp::shutdown();
	return 0;
} 
