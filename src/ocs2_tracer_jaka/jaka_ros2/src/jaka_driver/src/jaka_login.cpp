#include "JAKAZuRobot.h"
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;

JAKAZuRobot robot;

int main(void)
{
    setlocale(LC_ALL, "");
    // robot.login_in(argv[1]);
    string default_ip = "10.5.5.100";
    robot.login_in(default_ip.c_str());
    robot.power_on();
    cout << "robot login in" << endl;
    std::this_thread::sleep_for(std::chrono::seconds(8));
    robot.enable_robot();
    cout << "robot enable" << endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    //Joint-space first-order low-pass filtering in robot servo mode
    robot.servo_move_use_joint_LPF(2);
    
    robot.set_torque_sensor_mode(1); 
      
    // robot.servo_move_use_joint_NLF(45,30,30);
    //robot.servo_move_use_joint_NLF(45,45,45);
    std::cout << "you can start jaka" << std::endl;
    return 0;
}
