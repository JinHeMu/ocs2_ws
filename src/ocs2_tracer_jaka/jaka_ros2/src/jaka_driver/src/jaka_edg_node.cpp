#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// 引入 JAKA SDK 头文件
#include "JAKAZuRobot.h"
#include "jktypes.h"

#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

#include <fstream> // [新增] 文件流头文件

class JakaEdgNode : public rclcpp::Node
{
public:
    JakaEdgNode() : Node("jaka_edg_node"), is_connected_(false)
    {
        // 1. 声明参数
        this->declare_parameter<std::string>("robot_ip", "10.5.5.100");
        this->declare_parameter<std::string>("local_ip", "10.5.5.127");
        
        // [新增] 声明采集开关和文件名参数
        this->declare_parameter<bool>("record_data", true);
        this->declare_parameter<std::string>("csv_path", "gravity_data.csv");

        // 2. 初始化 SDK 对象
        robot_ = std::make_shared<JAKAZuRobot>();

        // 3. 创建发布者
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("jaka_joint_states", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("jaka_tcp_pose", 10);

        // 4. 启动一个独立线程进行机器人连接和初始化，防止阻塞 ROS 主线程
        init_thread_ = std::thread(&JakaEdgNode::init_robot_sequence, this);

                // [新增] 打开 CSV 文件并写入表头
        bool record = this->get_parameter("record_data").as_bool();
        if (record) {
            std::string path = this->get_parameter("csv_path").as_string();
            csv_file_.open(path, std::ios::out | std::ios::trunc);
            if (csv_file_.is_open()) {
                // 写入表头：6个关节角 + 6个力/力矩
                csv_file_ << "joint0,joint1,joint2,joint3,joint4,joint5,"
                          << "fx,fy,fz,tx,ty,tz\n";
                RCLCPP_INFO(this->get_logger(), "数据记录已开启，文件路径: %s", path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", path.c_str());
            }
        }

        // 5. 创建定时器 (100ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&JakaEdgNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "节点已启动，正在后台连接机器人...");
    }

    ~JakaEdgNode()
    {
        if (init_thread_.joinable()) {
            init_thread_.join();
        }
        
        if (robot_) {
            RCLCPP_INFO(this->get_logger(), "正在关闭 EDG 并断开连接...");
            robot_->edg_init(false); // 关闭 EDG
            robot_->login_out();
        }

                // [新增] 关闭文件
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "数据采集结束，文件已保存。");
        }
    }

private:
    // 将耗时的初始化过程放入独立函数
    void init_robot_sequence()
    {
        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        std::string local_ip = this->get_parameter("local_ip").as_string();

        RCLCPP_INFO(this->get_logger(), "线程: 正在连接 IP: %s", robot_ip.c_str());
        if (robot_->login_in(robot_ip.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "线程: 登录失败！请检查网线连接。");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "线程: 正在上电 (等待 4s)...");
        robot_->power_on();
        std::this_thread::sleep_for(std::chrono::seconds(4));
        
        RCLCPP_INFO(this->get_logger(), "线程: 正在使能 (等待 4s)...");
        robot_->enable_robot();
        std::this_thread::sleep_for(std::chrono::seconds(4));

        RCLCPP_INFO(this->get_logger(), "线程: 初始化 EDG UDP 流，本机 IP: %s", local_ip.c_str());
        int ret = robot_->edg_init(true, local_ip.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "线程: EDG 初始化失败，错误码: %d. 请检查防火墙和本机IP设置。", ret);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "线程: 机器人准备就绪！开始读取数据。");
        is_connected_ = true; // 设置标志位，允许定时器读取数据
    }

    void timer_callback()
    {
        // 如果还没连接好，直接跳过
        if (!is_connected_) return;

        EDGState state;
        // 获取 EDG 数据 (非阻塞或快速返回)
        int ret = robot_->edg_get_stat(&state);
        
        if (ret == 0) {
            auto now = this->get_clock()->now();

            // --- 发布关节数据 ---
            sensor_msgs::msg::JointState joint_msg;
            joint_msg.header.stamp = now;
            joint_msg.header.frame_id = "jaka_base_link";
            joint_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            
            // 批量赋值，避免在循环里打印日志
            joint_msg.position.assign(state.jointVal.jVal, state.jointVal.jVal + 6);
            joint_msg.velocity.assign(state.jointVel.jVel, state.jointVel.jVel + 6);
            joint_msg.effort.assign(state.jointTorq.jtorq, state.jointTorq.jtorq + 6);
            
            joint_pub_->publish(joint_msg);

            // --- 发布 TCP 位姿 ---
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = now;
            pose_msg.header.frame_id = "jaka_base_link";
            pose_msg.pose.position.x = state.cartpose.tran.x / 1000.0;
            pose_msg.pose.position.y = state.cartpose.tran.y / 1000.0;
            pose_msg.pose.position.z = state.cartpose.tran.z / 1000.0;

            tf2::Quaternion q;
            q.setRPY(state.cartpose.rpy.rx, state.cartpose.rpy.ry, state.cartpose.rpy.rz);
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pose_pub_->publish(pose_msg);

            // --- 关键修改：每 2 秒才打印一次状态，防止终端卡死 ---
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "EDG 数据正常: J1: %.2f, X: %.2f, Y: %.2f, Z: %.2f", 
                joint_msg.position[0], pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

                // --- 2. [新增] 核心采集逻辑：写入 CSV ---
            if (csv_file_.is_open()) {
                // 写入 6 个关节角 (弧度)
                for (int i = 0; i < 6; i++) {
                    csv_file_ << state.jointVal.jVal[i] << ",";
                }

                // 写入 6 维力传感器数据
                // 注意：请在 jktypes.h 中确认字段名，通常是 torqSensor 或 extSensor
                // 假设单位：力是 N，力矩是 Nm
                csv_file_ << state.torqSensor.fx << ","
                            << state.torqSensor.fy << ","
                            << state.torqSensor.fz << ","
                            << state.torqSensor.tx << ","
                            << state.torqSensor.ty << ","
                            << state.torqSensor.tz << "\n"; // 换行
            }

        }
        
         else {
            // 出错时也限制打印频率
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "获取 EDG 数据失败 (Ret: %d). 请检查防火墙是否允许 UDP 端口 10000-10010.", ret);
        }

        
    }

    std::shared_ptr<JAKAZuRobot> robot_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::thread init_thread_;   // 用于后台初始化
    std::atomic<bool> is_connected_; // 线程安全的标志位
    std::ofstream csv_file_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JakaEdgNode>());
    rclcpp::shutdown();
    return 0;
}
