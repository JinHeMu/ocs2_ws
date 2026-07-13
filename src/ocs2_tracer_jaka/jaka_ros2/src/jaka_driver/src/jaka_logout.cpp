#include "JAKAZuRobot.h"
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;

JAKAZuRobot robot;

int main(void)
{

    string default_ip = "10.5.5.100";
    robot.login_in(default_ip.c_str());
    robot.disable_robot();
    robot.power_off();
    robot.shut_down();

    std::cout << "robot disable" << std::endl;
    return 0;
}
